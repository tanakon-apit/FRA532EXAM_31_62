
#ifndef __KINEMMATIC_H__
#define __KINEMMATIC_H__

typedef struct
{
    float left_wheel;
    float right_wheel;
    float a0;
    float a1;
} DiffDriveKinematic;

void DiffDrive_IK_Init(DiffDriveKinematic *kine, float r, float b);

void DiffDrive_IK_Compute(DiffDriveKinematic *kine, double vx, double wz);

#endif