// Inverse kinematics for the SO-101 with the gripper pointing straight down.
//
// After shoulder_pan, the arm is a planar 3-link chain. I measured each link
// at the zero pose in the model (see measure.py). Each link is a length plus
// the angle it makes with the floor when all joints are at 0.
//
// Build: make  (gives libik.so, which ik.py loads with ctypes)

#include <math.h>

#define PAN_X 0.03884   // pan axis, x position in the world
#define SH_R  0.03040   // shoulder_lift pivot, measured from the pan axis
#define SH_Z  0.11660

#define L1 0.11600      // shoulder_lift -> elbow_flex
#define A1 1.32701
#define L2 0.13500      // elbow_flex -> wrist_flex
#define A2 0.03853
#define L3 0.15920      // wrist_flex -> tip site (in line with the wrist)

// joint limits from so101.xml
static const double lo[4] = {-1.9199, -1.7453, -1.69, -1.6581};
static const double hi[4] = { 1.9199,  1.7453,  1.69,  1.6581};

// x, y, z is where the gripper tip should go (world frame, meters).
// Writes shoulder_pan, shoulder_lift, elbow_flex, wrist_flex into q.
// Returns 0 on success, -1 if the point is out of reach or past a joint limit.
int so101_ik(double x, double y, double z, double *q)
{
    // pan joint axis points down, so the angle is flipped
    double pan = -atan2(y, x - PAN_X);
    double r = sqrt((x - PAN_X) * (x - PAN_X) + y * y);

    // gripper points down, so the wrist is right above the tip
    double wr = r - SH_R;
    double wz = z + L3 - SH_Z;

    // two-link IK from the shoulder to the wrist (law of cosines)
    double d2 = wr * wr + wz * wz;
    double c = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    if (c > 1.0 || c < -1.0)
        return -1;
    double bend = -acos(c);  // elbow up

    double t1 = atan2(wz, wr) - atan2(L2 * sin(bend), L1 + L2 * cos(bend));
    double t2 = t1 + bend;
    double t3 = -M_PI / 2;

    // joint angles. +rotation about the joint axis tilts the link down
    q[0] = pan;
    q[1] = A1 - t1;
    q[2] = A2 - t2 - q[1];
    q[3] = -t3 - q[1] - q[2];

    for (int i = 0; i < 4; i++)
        if (q[i] < lo[i] || q[i] > hi[i])
            return -1;
    return 0;
}
