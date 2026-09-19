std_msgs/Header header
# Fresh motor feedback, not merely existence of the USB adapter.
bool connected
# Healthy link and no motion-inhibit latch; individual commands can still be busy/rejected.
bool ready
bool ever_connected
bool recovery_required
# not_connected, disconnected, recovery_required, ready
string status
string detail
float64 sample_age_s
