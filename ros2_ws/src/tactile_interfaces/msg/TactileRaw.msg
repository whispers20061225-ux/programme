std_msgs/Header header

uint32 rows
uint32 cols
uint64 sequence_id
string frame_id

# Flattened tactile values of size rows * cols.
float32[] forces

# Optional per-taxel 3D force components of size rows * cols.
float32[] forces_fx
float32[] forces_fy
float32[] forces_fz

# Optional per-taxel 3D torque components of size rows * cols.
float32[] torques_tx
float32[] torques_ty
float32[] torques_tz

# Decoded total force and contact state for the current tactile frame.
float32 total_fx
float32 total_fy
float32 total_fz
bool contact_active
float32 contact_score
bool baseline_ready
string sensor_layout

# Source and diagnostics metadata.
string requested_mode
string active_mode
string source_name
bool source_connected
uint32 device_addr
uint32 function_code
uint32 start_addr
uint32 returned_data_len_hint
uint32 frame_len_declared
uint32 payload_len_observed
uint32 payload_value_count
float32 transport_rate_hz
float32 publish_rate_hz
bool lrc_ok
string parser_state
string mapping_state
string status_text
string raw_frame_hex
