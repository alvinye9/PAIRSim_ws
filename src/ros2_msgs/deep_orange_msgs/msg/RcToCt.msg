builtin_interfaces/Time stamp
uint8 rolling_counter
# flag signal fields are shared by both MyLaps and Marelli race control system
uint8 track_flag
uint8 vehicle_flag
uint8 sector_flag
# LTE statuses are only filled in by Marelli race control system
uint8 lte_rssi
bool lte_sync_ok
uint8 vehicle_rank
uint8 lap_count
float32 lap_distance
float32 round_target_speed
