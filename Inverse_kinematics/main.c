#include <stdio.h>
#include <math.h>

//Calculating Angles Based on x, y, z
//This is just one way of calculating the joint angles based on coordinate
//(x,y,z) of the end of the gripper. Note that the equations need to be
//calculated in RADIANS (not degrees) and then converted back to degrees. We
//also suggest including constraints to ensure the given point can be reached
//by the arm.
//
//L is the length of the shoulder axis to the elbow axis
//M is the length of the elbow axis to the wrist axis
//N is the length of the wrist axis to the end of the gripper (or the desired point)
//
//R = (x^2) + (y^2)
//Represents the radius from the axis of rotation of the base to x,y
//
//s = R – N
//Since the arm has four degrees of freedom, there are infinite solutions
//possible for the arm to reach point (x,y,z). We will therefore introduce an
//artificial constraint and keep the gripper at a specific angle to the
//horizontal, and calculate for a new coordinate (x1, y1, z1) of the wrist axis.
//
//Q = [(s^2) + (z^2)]^(1/2)
//This is the distance between the shoulder axis and the wrist axis
//
//f = atan2(z, s)
//This is the angle between the horizontal and the line Q. The atan function
//would return two angles whereas the atan2 function determines the correct
//angle based on the x and y coordinate. The actual height is h + z which can
//be taken into account when inputting
//
//g = acos[ ((L^2)+(Q^2)-(M^2)) / (2*L*Q)]
//This is the angle between line Q and link L using the law of cosines.
//Use the equations above to find angles a, b, c and d:
//
//a = f + g
//This is angle 'a' above.
//
//b = acos[((M^2)+(L^2)-(Q^2)) / (2*L*M)]
//This is angle ‘b’ above using the law of cosines.
//
//c = -b - a + 2*pi
//This is angle 'c'. Angle c is kept horizontal to the (x,y) plane.
//
//d = math.atan2(x, y)
//This is the angle 'd' of the base. 
//
// For more info: https://www.robotshop.com/media/files/PDF/robotshop-multi-purpose-robotic-arm-guide.pdf


#define M_PI   3.14159265358979323846264338327950288

#define deg2Rad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define rad2Deg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

int main()
{

    float L,M,N,h;          // Links mm, L = shoulder, M = elbow, N = wrist
    float dx, dy, dz;       // EE position
    float theta1, theta2, theta3, theta4;   // Joint variables that will be calculated
    float A, B, C;

    float R,S,Q,f,g;

    h = 170;    // height
    L = 45;     // shoulder
    M = 85;     // elbow
    N = 130;    // wrist

    // Global EE coordinates
    dx = 3.0;
    dy = 5.0;
    dz = 2.0;

    R = pow(dx,2) + pow(dy,2);  // The radius from the axis of rotation of the base to dx,dy
    S = R - N;                  // 
    Q = sqrt(pow(S, 2) + pow(dz, 2));
    f = atan2(dz+h, S);
    g = acos((pow(L,2)+pow(Q,2)-pow(M,2)) /(2*L*Q));

    // Angles
    // theta1 = Base
    // theta2 = Shoulder
    // theta3 = Elbow
    // theta4 = Wrist
    theta1 = atan2(dx, dy);
    theta2 = f + g;
    theta3 = acos((pow(M, 2) + pow(L, 2) - pow(Q, 2)) / (2 * L * M));
    theta4 = -theta3 - theta2 + 2 * M_PI;

    printf("R: %f\n", R);
    printf("Q: %f\n", Q);
    printf("f: %f\n", rad2Deg(f));
    printf("g: %f\n", rad2Deg(g));
    printf("Base: theta1: %f\n", rad2Deg(theta1));
    printf("Shoulder theta2: %f\n", rad2Deg(theta2));
    printf("Elbow theta3: %f\n", rad2Deg(theta3));
    printf("Wrist theta4: %f\n", rad2Deg(theta4));

    return 0;
}
