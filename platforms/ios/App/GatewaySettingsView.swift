import MotoNavigationCore
import SwiftUI

struct GatewaySettingsView: View {
    @ObservedObject var model: AppModel
    @Environment(\.dismiss) private var dismiss
    @State private var address = ""
    @State private var message: String?
    @State private var isChecking = false
    @State private var checkTask: Task<Void, Never>?

    var body: some View {
        NavigationStack {
            Form {
                Section {
                    TextField("https://你的网关地址/", text: $address, axis: .vertical)
                        .keyboardType(.URL)
                        .textInputAutocapitalization(.never)
                        .autocorrectionDisabled()
                        .accessibilityIdentifier("gateway-address-field")
                        .disabled(model.isUpdatingGateway)
                } header: {
                    Text("网关地址")
                } footer: {
                    Text("填写完整的 HTTPS 根地址，例如 https://1.94.65.14/moto-gps/api/。不要填高德 Key。")
                }
                Section {
                    Button {
                        checkConnection()
                    } label: {
                        HStack {
                            Text("测试连接")
                            if isChecking { Spacer(); ProgressView() }
                        }
                    }
                    .disabled(isChecking || model.isUpdatingGateway || address.isEmpty)
                    .accessibilityIdentifier("gateway-check-button")
                    if let message {
                        Text(message).foregroundStyle(.secondary)
                            .accessibilityIdentifier("gateway-status-message")
                    }
                } footer: {
                    Text("连接测试只检查网关状态；真实搜索、导航和地图还需要相应服务可用。")
                }
                Section {
                    Text("保存后立即生效，无需重新安装。更换地址会清空当前搜索并暂停地图下载，已下载的地图仍会保留。")
                    Text("地点、路线和定位请求将发送到这个地址，请使用你信任的网关。")
                }
                if model.isNavigationActive {
                    Section { Text("请先结束导航，再更换网关地址。") }
                }
            }
            .navigationTitle("网关设置")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("取消") { dismiss() }.disabled(model.isUpdatingGateway)
                }
                ToolbarItem(placement: .confirmationAction) {
                    Button("保存") {
                        checkTask?.cancel()
                        isChecking = false
                        Task {
                            do {
                                try await model.saveGatewayAddress(address)
                                dismiss()
                            } catch { message = error.localizedDescription }
                        }
                    }
                    .disabled(model.isNavigationActive || model.isUpdatingGateway || address.isEmpty)
                    .accessibilityIdentifier("gateway-save-button")
                }
            }
            .interactiveDismissDisabled(model.isUpdatingGateway)
            .onAppear { address = model.isGatewayConfigured ? model.mapGatewayBaseURL.absoluteString : "" }
            .onChange(of: address) { _, _ in
                checkTask?.cancel()
                isChecking = false
                message = nil
            }
            .onDisappear { checkTask?.cancel() }
        }
    }

    private func checkConnection() {
        checkTask?.cancel()
        message = nil
        let url: URL
        do { url = try GatewayConfiguration.normalizedURL(address) }
        catch { message = error.localizedDescription; return }
        isChecking = true
        checkTask = Task { @MainActor in
            defer { if !Task.isCancelled { isChecking = false } }
            do {
                var request = URLRequest(url: url.appendingPathComponent("healthz"))
                request.timeoutInterval = 10
                request.cachePolicy = .reloadIgnoringLocalCacheData
                let (bytes, response) = try await URLSession.shared.bytes(for: request)
                defer { bytes.task.cancel() }
                guard let response = response as? HTTPURLResponse, response.statusCode == 200,
                      response.expectedContentLength <= 65_536 else { throw URLError(.badServerResponse) }
                var data = Data()
                for try await byte in bytes {
                    guard data.count < 65_536 else { throw URLError(.dataLengthExceedsMaximum) }
                    data.append(byte)
                }
                struct Health: Decodable {
                    let status: String
                    let ready_for_live_navigation: Bool
                }
                let health = try JSONDecoder().decode(Health.self, from: data)
                guard health.status == "ok" else { throw URLError(.badServerResponse) }
                try Task.checkCancellation()
                message = health.ready_for_live_navigation
                    ? "网关已连接，已启用实时导航配置。"
                    : "网关已连接，但尚未启用实时导航。请检查服务端配置。"
            } catch {
                guard !Task.isCancelled else { return }
                message = "连接失败，请检查地址、HTTPS 证书和网关是否已启动。"
            }
        }
    }
}
