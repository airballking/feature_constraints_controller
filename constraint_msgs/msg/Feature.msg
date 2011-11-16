# tf frame name. pos, dir and ref_dir are expressed
# in this frame
string frame_id

# name or id assigned by the knowledge base
string name

uint8 LINE=0
uint8 PLANE=1
# other features: concavities, holes, ...
uint8 type
geometry_msgs/Vector3 pos  # center position of the feature
geometry_msgs/Vector3 dir  # direction of the feature. length encodes size.
geometry_msgs/Vector3 ref_dir # reference (zero) direction. hardly ever used.

