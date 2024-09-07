# Match.msg
# meta data
string id
string title

# team color
int32 UNKNOWN = 0
int32 RED = 1
int32 BLUE = 2

builtin_interfaces/Time start_time
builtin_interfaces/Time end_time

game_state_interfaces/Team red_team
game_state_interfaces/Team blue_team

int32 winner
