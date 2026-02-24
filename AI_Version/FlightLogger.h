#pragma once
#include <vector>
#include <string>

/**
 * FlightLogger
 * ------------
 * Collects per-timestep flight data into parallel vectors and writes them
 * out as a CSV file at the end of the simulation.
 *
 * Callers fill in a  Sample  struct and hand it to  record():
 *
 *   FlightLogger::Sample s;
 *   s.time = state.time;
 *   s.positionZ = state.position[2];
 *   // ...fill remaining fields...
 *   logger.record(s);
 *
 * Then at the end:
 *   logger.saveCSV("output.csv");
 */
class FlightLogger {
public:
    /** One data-point snapshot, populated externally and handed to record(). */
    struct Sample {
        double time              = 0;
        double positionX         = 0;
        double positionY         = 0;
        double positionZ         = 0;
        double velocityMag       = 0;
        double verticalVelocity  = 0;
        double horizontalVelocity= 0;
        double accelerationMag   = 0;
        double roll              = 0;   // deg
        double pitch             = 0;   // deg
        double yaw               = 0;   // deg
        double rollRate          = 0;   // deg/s
        double pitchRate         = 0;
        double yawRate           = 0;
        double thrust            = 0;   // N
        double drag              = 0;   // N
        double mass              = 0;   // kg
        double reqRollRate       = 0;
        double reqPitchRate      = 0;
        double reqYawRate        = 0;
        double reqRollCtrl       = 0;
        double reqPitchCtrl      = 0;
        double reqYawCtrl        = 0;
        double fin1              = 0;   // deg
        double fin2              = 0;
        double fin3              = 0;
        double fin4              = 0;
        double machNumber        = 0;
        double dynamicPressure   = 0;   // Pa
    };

    /** Appends a sample to the internal buffers. */
    void record(const Sample& s);

    /** Writes all recorded samples to a CSV file. Returns true on success. */
    bool saveCSV(const std::string& filename) const;

    /** Clears all buffered data. */
    void clear();

    std::size_t sampleCount() const { return time_.size(); }

private:
    // Parallel vectors — one entry per call to record()
    std::vector<double>
        time_, posX_, posY_, posZ_,
        velMag_, velVert_, velHoriz_,
        accelMag_,
        roll_, pitch_, yaw_,
        rollRate_, pitchRate_, yawRate_,
        thrust_, drag_, mass_,
        reqRollRate_, reqPitchRate_, reqYawRate_,
        reqRollCtrl_, reqPitchCtrl_, reqYawCtrl_,
        fin1_, fin2_, fin3_, fin4_,
        mach_, dynQ_;
};
