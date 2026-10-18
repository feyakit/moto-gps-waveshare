import XCTest
@testable import MotoNavigationCore

final class TripComputerTests: XCTestCase {
    private let origin = WGS84Point(longitudeDeg: 117.0, latitudeDeg: 36.65)

    private func fix(
        at seconds: TimeInterval,
        metresNorth: Double = 0,
        metresEast: Double = 0,
        altitudeM: Double? = nil,
        accuracyM: Double = 5,
        speedMps: Double? = nil
    ) -> NavigationFix {
        // 1 degree of latitude is ~111.32 km; longitude shrinks by cos(lat).
        let latitudeDeg = origin.latitudeDeg + metresNorth / 111_320
        let metresPerLongitudeDegree = 111_320 * cos(origin.latitudeDeg * .pi / 180)
        let longitudeDeg = origin.longitudeDeg + metresEast / metresPerLongitudeDegree
        return NavigationFix(
            coordinate: WGS84Point(longitudeDeg: longitudeDeg, latitudeDeg: latitudeDeg),
            altitudeM: altitudeM,
            horizontalAccuracyM: accuracyM,
            speedMps: speedMps,
            courseDeg: 0,
            timestamp: Date(timeIntervalSince1970: 1_700_000_000 + seconds)
        )
    }

    func testDistanceAndMovingTimeAccumulateWhileRiding() {
        let computer = TripComputer()
        computer.accept(fix(at: 0, speedMps: 0))
        computer.accept(fix(at: 1, metresNorth: 14, speedMps: 14))
        computer.accept(fix(at: 2, metresNorth: 28, speedMps: 14))

        let totals = computer.totals
        XCTAssertTrue(totals.isRecording)
        // Measured on the sphere, so the local-metre fixture lands 1 m short.
        XCTAssertEqual(Double(totals.distanceM), 28, accuracy: 1)
        XCTAssertEqual(totals.movingTimeS, 2)
        XCTAssertEqual(Double(totals.maxSpeedDeciKph), 504, accuracy: 1)
    }

    func testParkedJitterNeverAddsDistanceOrMovingTime() {
        let computer = TripComputer()
        for step in 0 ... 60 {
            // Stationary GNSS noise: a few metres of wander and speed 0.
            let wander = Double((step % 3) - 1) * 3
            computer.accept(fix(at: Double(step), metresNorth: wander, speedMps: 0))
        }

        XCTAssertEqual(computer.totals.distanceM, 0)
        XCTAssertEqual(computer.totals.movingTimeS, 0)
    }

    func testMovingHysteresisIgnoresSpeedsBetweenTheThresholds() {
        let computer = TripComputer()
        computer.accept(fix(at: 0, speedMps: 0))
        // 1.0 m/s is fast enough to look like movement but below the start
        // threshold, so the clock must not run yet.
        computer.accept(fix(at: 1, metresNorth: 1, speedMps: 1.0))
        XCTAssertEqual(computer.totals.movingTimeS, 0)

        computer.accept(fix(at: 2, metresNorth: 4, speedMps: 3.0))
        XCTAssertEqual(computer.totals.movingTimeS, 1)

        // Once moving, 1.0 m/s is above the stop threshold, so the clock
        // keeps running through low-speed traffic.
        computer.accept(fix(at: 3, metresNorth: 5, speedMps: 1.0))
        XCTAssertEqual(computer.totals.movingTimeS, 2)
    }

    func testInaccurateFixIsIgnoredEntirely() {
        let computer = TripComputer()
        computer.accept(fix(at: 0, speedMps: 10))
        computer.accept(fix(at: 1, metresNorth: 50, accuracyM: 120, speedMps: 10))

        // A 120 m fix can wander 120 m without the bike moving, so it must not
        // touch the odometer even though the first fix started the trip.
        XCTAssertEqual(computer.totals.distanceM, 0)
        XCTAssertEqual(computer.totals.movingTimeS, 0)
        XCTAssertTrue(computer.totals.isRecording)
    }

    func testElevationGainCountsRisesOnce() {
        var configuration = TripComputerConfiguration()
        configuration.altitudeSmoothing = 1.0
        let computer = TripComputer(configuration: configuration)
        let altitudes: [Double] = [100, 104, 103, 108, 101, 107]
        for (index, altitude) in altitudes.enumerated() {
            computer.accept(
                fix(at: Double(index), metresNorth: Double(index) * 20,
                    altitudeM: altitude, speedMps: 20)
            )
        }

        XCTAssertTrue(computer.totals.elevationKnown)
        // 100 -> 104 books 4, the 1 m dip resets the reference, 104 -> 108
        // books 5, the drop to 101 resets again, and 101 -> 107 books 6.
        XCTAssertEqual(computer.totals.elevationGainM, 15)
    }

    func testLongGapRebasesWithoutInventingDistance() {
        let computer = TripComputer()
        computer.accept(fix(at: 0, speedMps: 10))
        // A tunnel: 120 s later the rider is 2 km further along. The straight
        // line is not the path, so nothing is booked but the baseline moves.
        computer.accept(fix(at: 120, metresNorth: 2_000, speedMps: 20))
        XCTAssertEqual(computer.totals.distanceM, 0)
        XCTAssertEqual(computer.totals.movingTimeS, 0)

        computer.accept(fix(at: 121, metresNorth: 2_020, speedMps: 20))
        XCTAssertEqual(Double(computer.totals.distanceM), 20, accuracy: 1)
        XCTAssertEqual(computer.totals.movingTimeS, 1)
    }

    func testTeleportSpikeIsRejected() {
        let computer = TripComputer()
        computer.accept(fix(at: 0, speedMps: 10))
        computer.accept(fix(at: 1, metresNorth: 5_000, speedMps: 10))

        // The step implies 5 km/s, which is not a motorcycle.
        XCTAssertEqual(computer.totals.distanceM, 0)
    }

    func testResetClearsEveryTotal() {
        let computer = TripComputer()
        computer.accept(fix(at: 0, altitudeM: 100, speedMps: 10))
        computer.accept(fix(at: 1, metresNorth: 30, altitudeM: 106, speedMps: 30))
        XCTAssertGreaterThan(computer.totals.distanceM, 0)

        let cleared = computer.reset()
        XCTAssertEqual(cleared, TripTotals())
        XCTAssertEqual(computer.totals, TripTotals())
    }

    func testFreshComputerReportsNotRecording() {
        let computer = TripComputer()
        XCTAssertFalse(computer.totals.isRecording)
        XCTAssertFalse(computer.totals.elevationKnown)
        XCTAssertEqual(computer.totals, TripTotals())
    }
}
