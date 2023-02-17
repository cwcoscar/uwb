time stamp
# receiving time

string MID
# mr: distance from tag to anchor
# mc: distance from tag to anchor, being optimized
# ma: distance from anchor to anchor

int32 Tag_id
# tag id of receiving data, distance from this tag to each available anchor

bool A0
bool A1
bool A2
bool A3
# avaliable anchor 

float32 distance_to_A0
float32 distance_to_A1
float32 distance_to_A2
float32 distance_to_A3
# receiving data of certain tag to each anchor