import Foundation

/// Cumulative ride totals for the round display's Trip page.
///
/// The phone is the only device that sees every GNSS fix, so it owns the
/// accumulation. The board displays these numbers and never derives distance
/// of its own; that keeps a single source of truth for a rider who moves
/// between the phone and the handlebar gauge.
public struct TripTotals: Equatable, Sendable {
    public var distanceM: UInt32
    public var movingTimeS: UInt32
    public var elevationGainM: UInt32
    public var maxSpeedDeciKph: UInt16
    /// False until the first usable fix arrives; the board then shows the
    /// empty page instead of four zeros that look like a finished ride.
    public var isRecording: Bool
    /// True once a smoothed altitude is available, so a barometer-less phone
    /// riding on flat ground does not claim "0 m of climbing" as a fact.
    public var elevationKnown: Bool

    public init(
        distanceM: UInt32 = 0,
        movingTimeS: UInt32 = 0,
        elevationGainM: UInt32 = 0,
        maxSpeedDeciKph: UInt16 = 0,
        isRecording: Bool = false,
        elevationKnown: Bool = false
    ) {
        self.distanceM = distanceM
        self.movingTimeS = movingTimeS
        self.elevationGainM = elevationGainM
        self.maxSpeedDeciKph = maxSpeedDeciKph
        self.isRecording = isRecording
        self.elevationKnown = elevationKnown
    }
}

/// Tunables that decide what counts as "moving". The defaults are chosen for
/// street riding and match the thresholds used elsewhere in the app.
public struct TripComputerConfiguration: Sendable {
    /// Fixes worse than this are ignored entirely; a 30 m fix can move the
    /// odometer by 30 m without the bike moving at all.
    public var maximumHorizontalAccuracyM: Double = 30
    /// Speed that starts the moving clock.
    public var movingStartSpeedMps: Double = 1.5
    /// Speed that stops it. The gap to `movingStartSpeedMps` is hysteresis so
    /// GPS noise at a red light cannot start and stop the clock every second.
    public var movingStopSpeedMps: Double = 0.8
    /// Steps implying a faster speed than this are treated as GPS spikes.
    public var maximumPlausibleSpeedMps: Double = 100
    /// A longer gap between fixes means the track is unknown; the step is used
    /// only to rebase, so a tunnel never adds a straight-line shortcut.
    public var maximumGapS: Double = 15
    /// Minimum rise before an ascent is booked, and the descent that resets
    /// the reference so the next climb is measured from the valley floor.
    public var minimumElevationRiseM: Double = 1
    /// Exponential smoothing applied to the raw GNSS altitude.
    public var altitudeSmoothing: Double = 0.3

    public init() {}
}

/// Accumulates `TripTotals` from a stream of `NavigationFix` values.
///
/// The type is deliberately dependency-free and synchronous so the same
/// numbers can be asserted in unit tests without a CoreLocation harness.
public final class TripComputer {
    public private(set) var totals: TripTotals

    private let configuration: TripComputerConfiguration
    private var previousFix: NavigationFix?
    private var distanceAccumulatorM: Double = 0
    private var movingTimeAccumulatorS: Double = 0
    private var elevationGainAccumulatorM: Double = 0
    private var smoothedAltitudeM: Double?
    private var elevationReferenceM: Double?
    private var isMoving = false

    public init(configuration: TripComputerConfiguration = TripComputerConfiguration()) {
        self.configuration = configuration
        totals = TripTotals()
    }

    /// Feeds one fix and returns the updated totals.
    @discardableResult
    public func accept(_ fix: NavigationFix) -> TripTotals {
        guard fix.coordinate.isValid,
              fix.horizontalAccuracyM >= 0,
              fix.horizontalAccuracyM <= configuration.maximumHorizontalAccuracyM
        else { return totals }

        totals.isRecording = true
        updateAltitude(fix.altitudeM)

        guard let previous = previousFix else {
            previousFix = fix
            updateMaxSpeed(fix.speedMps)
            return publish()
        }

        let elapsedS = fix.timestamp.timeIntervalSince(previous.timestamp)
        defer {
            previousFix = fix
        }
        guard elapsedS > 0, elapsedS <= configuration.maximumGapS else {
            // Too long a gap to know the path. Rebase only.
            updateMaxSpeed(fix.speedMps)
            return publish()
        }

        let stepM = Self.distanceM(from: previous.coordinate, to: fix.coordinate)
        let derivedSpeedMps = stepM / elapsedS
        let speedMps = Self.plausibleSpeed(fix.speedMps) ?? Self.plausibleSpeed(derivedSpeedMps)
        updateMaxSpeed(speedMps)

        // Moving state uses hysteresis, so a fix either confirms the current
        // state or has to clear the wider threshold to flip it.
        if isMoving {
            if let speedMps, speedMps < configuration.movingStopSpeedMps {
                isMoving = false
            }
        } else if let speedMps, speedMps >= configuration.movingStartSpeedMps {
            isMoving = true
        }

        guard isMoving else {
            return publish()
        }

        // Distance is only booked while moving. Parked GPS jitter walks a few
        // metres per second of noise and would otherwise add kilometres to a
        // bike sitting in a garage.
        if stepM <= configuration.maximumPlausibleSpeedMps * elapsedS {
            distanceAccumulatorM += stepM
        }
        movingTimeAccumulatorS += elapsedS
        return publish()
    }

    /// Zeroes every total, keeping the configuration.
    @discardableResult
    public func reset() -> TripTotals {
        previousFix = nil
        distanceAccumulatorM = 0
        movingTimeAccumulatorS = 0
        elevationGainAccumulatorM = 0
        smoothedAltitudeM = nil
        elevationReferenceM = nil
        isMoving = false
        totals = TripTotals()
        return totals
    }

    private func publish() -> TripTotals {
        // Totals are truncated once, when published, rather than on every fix.
        // Truncating each step would silently drop up to a metre per fix of
        // slow riding, and a 1 Hz stream would under-count by up to 3.6 km/h.
        totals.distanceM = Self.clamped(distanceAccumulatorM)
        totals.movingTimeS = UInt32(movingTimeAccumulatorS.rounded(.down))
        totals.elevationGainM = UInt32(elevationGainAccumulatorM.rounded(.down))
        return totals
    }

    private func updateMaxSpeed(_ speedMps: Double?) {
        guard let speedMps = Self.plausibleSpeed(speedMps) else { return }
        let deciKph = UInt16((speedMps * 36).rounded())
        if deciKph > totals.maxSpeedDeciKph {
            totals.maxSpeedDeciKph = deciKph
        }
    }

    private func updateAltitude(_ altitudeM: Double?) {
        guard let altitudeM, altitudeM.isFinite,
              altitudeM > -500, altitudeM < 9_000
        else { return }

        guard let current = smoothedAltitudeM else {
            smoothedAltitudeM = altitudeM
            elevationReferenceM = altitudeM
            return
        }
        let smoothed = current + (altitudeM - current) * configuration.altitudeSmoothing
        smoothedAltitudeM = smoothed
        totals.elevationKnown = true

        guard let reference = elevationReferenceM else {
            elevationReferenceM = smoothed
            return
        }
        let rise = smoothed - reference
        if rise >= configuration.minimumElevationRiseM {
            elevationGainAccumulatorM += rise
            elevationReferenceM = smoothed
        } else if -rise >= configuration.minimumElevationRiseM {
            // Descended far enough to forget the old summit; the next climb
            // must be measured from here rather than counted as the first one.
            elevationReferenceM = smoothed
        }
    }

    private static func plausibleSpeed(_ speedMps: Double?) -> Double? {
        guard let speedMps, speedMps.isFinite, speedMps >= 0, speedMps < 150 else {
            return nil
        }
        return speedMps
    }

    private static func clamped(_ metres: Double) -> UInt32 {
        guard metres.isFinite, metres > 0 else { return 0 }
        return metres >= Double(UInt32.max) ? UInt32.max : UInt32(metres)
    }

    /// Great-circle distance between two WGS-84 points.
    static func distanceM(from start: WGS84Point, to end: WGS84Point) -> Double {
        let earthRadiusM = 6_371_000.0
        let latitude1 = start.latitudeDeg * .pi / 180
        let latitude2 = end.latitudeDeg * .pi / 180
        let deltaLatitude = latitude2 - latitude1
        let deltaLongitude = (end.longitudeDeg - start.longitudeDeg) * .pi / 180
        let haversine = sin(deltaLatitude / 2) * sin(deltaLatitude / 2) +
            cos(latitude1) * cos(latitude2) *
            sin(deltaLongitude / 2) * sin(deltaLongitude / 2)
        return 2 * earthRadiusM * asin(min(1, sqrt(haversine)))
    }
}
