int32 NO_FAULT = 0
int32 SAFE_FAULT = 1
int32 LATENT_FAULT = 2
int32 SINGLE_POINT_FAULT = 3

int32 level
bool emergency
bool emergency_holding
diagnostic_msgs/DiagnosticStatus[] diagnostics_nf  # diagnostics categorized as no fault
diagnostic_msgs/DiagnosticStatus[] diagnostics_sf  # diagnostics categorized as safe fault
diagnostic_msgs/DiagnosticStatus[] diagnostics_lf  # diagnostics categorized as latent fault
diagnostic_msgs/DiagnosticStatus[] diagnostics_spf # diagnostics categorized as single point fault
