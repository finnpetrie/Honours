

#ifndef SIGNEDDISTANCEFRACTALS_H
#define SIGNEDDISTANCEFRACTALS_H

#include "RaytracingShaderHelper.hlsli"
#include "SignedDistancePrimitives.hlsli"

//------------------------------------------------------------------


float4 quatSq(float4 q)
{
    float4 r;
    r.x = q.x * q.x - dot(q.yzw, q.yzw);
    r.yzw = 2 * q.x * q.yzw;
    return r;
}


float4 quatMult(float4 q1, float4 q2)
{
    float4 r;
    r.x = q1.x * q2.x - dot(q1.yzw, q2.yzw);
    r.yzw = q1.x * q2.yzw + q2.x * q1.yzw + cross(q1.yzw, q2.yzw);
    return r;
}

float sdGyroid(in float3 position) {
    return sin(position.x) * cos(position.y) + sin(position.y) * cos(position.x) + sin(position.z) * cos(position.x);
}

float juliaMap(in float3 p, out float3 oTrap, in float4 c)
{
    float4 z = float4 (p, 0.0);
    float md2 = 1.0;
    float mz2 = dot(z, z);

    float4 trap = float4(abs(z.xyz), dot(z, z));

    float n = 1.0;
    for (int i = 0; i < 8; i++)
    {
        // dz -> 2·z·dz, meaning |dz| -> 2·|z|·|dz|
        // Now we take thr 2.0 out of the loop and do it at the end with an exp2
        md2 *= mz2;
        // z  -> z^2 + c
        z = quatSq(z) + c;

        trap = min(trap, float4(abs(z.xyz), dot(z, z)));

        mz2 = dot(z, z);
        if (mz2 > 4.0) break;
        n += 1.0;
    }

    oTrap = trap;

    return 0.25 * sqrt(mz2 / md2) * exp2(-n) * log(mz2);  // d = 0.5·|z|·log|z| / |dz|
}
float sdQuaternionJuliaSet(in float3 position, float4 h, in float Scale = 2.0f) {
    float4 z = float4(position, 0.0);
    float4 c = float4(0.6, 0.6, 0.6, 0.6);
    c = float4(0, 0, 0, 0);
    float mag_z = dot(z, z);
    float4 z_prime = float4(1, 0, 0, 0);
    int Iterations = 6;
    float4 mag_dz = 1;
    for (int i = 0; i < Iterations; i++) {
        z = quatSq(z) + c;
        z_prime = 2 * quatMult(z, z_prime);
        mag_dz = dot(z_prime, z_prime);
        mag_z = dot(z, z);
        if (mag_z > 4) break;
    }

    return sqrt(mag_z) / (2 * sqrt(mag_dz)) * log(dot(z, z));
}
#endif // SIGNEDDISTANCEFRACTALS_H