std_msgs/Header header
bool connected
bool simulated
bool busy
string command_id
string phase
string detail
float64 sample_age_s
float64 position_rev
float64 velocity_rps
float64 torque_nm
# 0 = configured closed endpoint; 1 = configured open endpoint.
float64 opening_fraction
# Nominal until width_calibrated is true. Never clamped to hide overtravel.
float64 width_mm
bool width_calibrated
# NaN unless a measured N/Nm calibration was configured. Not a force sensor.
float64 estimated_force_n
bool force_calibrated
uint8 mode
uint8 fault
float64 voltage_v
float64 temperature_c
float64 last_transport_rtt_ms
