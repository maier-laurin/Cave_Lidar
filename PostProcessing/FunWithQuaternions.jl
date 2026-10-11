include("DualQuaternions.jl")
using .DualQuaternions  # Note the dot before the name


# we start out with a point in 3d space: (1,1,1)
point = point_at(1.0,1.0,1.0)
# we then define a translation (-1, 2, 1)
translation = DualQuaternion((-1.0,2.0,1.0))
# lets apply this translation to the point
res1 = translation * point * dual_conj(translation)
# this schould have brought us to (0,3,2) lets see
get_point(res1)
#now lets define a rotation 1. rotate 90* around z (yaw)
#but we use the point x = 10 because its easier for us
point = point_at(10.0,0.0,0.0)
rot = DualQuaternion(Quaternion(0,0,π/2), (0.0,0.0,0.0))
get_point(rot * point *dual_conj(rot))
#now add a 45° upwards rotation
rot = DualQuaternion(Quaternion(0,π/4,π/2), (0.0,0.0,0.0))
get_point(rot * point *dual_conj(rot))

