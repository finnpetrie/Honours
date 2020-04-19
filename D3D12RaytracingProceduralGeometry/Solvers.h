#pragma once
int SolveQuadric(float3 c, float2 s)
{
    float p, q, D;

    /* normal form: x^2 + px + q = 0 */

    p = c.y / (2 * c.z);
    q = c.x / c.z;

    D = p * p - q;

    if (IsZero(D))
    {
        s.x = -p;
        return 1;
    }
    else if (D < 0)
    {
        return 0;
    }
    else /* if (D > 0) */
    {
        float sqrt_D = sqrt(D);

        s.x = sqrt_D - p;
        s.y = -sqrt_D - p;
        return 2;
    }
}


int SolveCubic(float4 c, float3 s)
{
    int     i, num;
    float  sub;
    float  A, B, C;
    float  sq_A, p, q;
    float  cb_p, D;

    /* normal form: x^3 + Ax^2 + Bx + C = 0 */

    A = c.z / c.w;
    B = c.y / c.w;
    C = c.x / c.w;

    /*  substitute x = y - A/3 to eliminate quadric term:
    x^3 +px + q = 0 */

    sq_A = A * A;
    p = 1.0 / 3 * (-1.0 / 3 * sq_A + B);
    q = 1.0 / 2 * (2.0 / 27 * A * sq_A - 1.0 / 3 * A * B + C);

    /* use Cardano's formula */

    cb_p = p * p * p;
    D = q * q + cb_p;

    if (IsZero(D))
    {
        if (IsZero(q)) /* one triple solution */
        {
            s.x = 0;
            num = 1;
        }
        else /* one single and one float solution */
        {
            float u = cbrt(-q);
            s.x = 2 * u;
            s.y = -u;
            num = 2;
        }
    }
    else if (D < 0) /* Casus irreducibilis: three real solutions */
    {
        float phi = 1.0 / 3 * acos(-q / sqrt(-cb_p));
        float t = 2 * sqrt(-p);

        s.x = t * cos(phi);
        s.y = -t * cos(phi + M_PI / 3);
        s.z = -t * cos(phi - M_PI / 3);
        num = 3;
    }
    else /* one real solution */
    {
        float sqrt_D = sqrt(D);
        float u = cbrt(sqrt_D - q);
        float v = -cbrt(sqrt_D + q);

        s.x = u + v;
        num = 1;
    }

    /* resubstitute */

    sub = 1.0 / 3 * A;

    s -= sub;

    return num;
}


int SolveQuartic(Quartic coef)
{
    float4  coeffs;
    float  z, u, v, sub;
    float  A, B, C, D;
    float sq_A, p, q, r;
    int     i, num;

    /* normal form: x^4 + Ax^3 + Bx^2 + Cx + D = 0 */

    A = coef.c.w / coef.lastC;
    B = coef.c.z / coef.lastC;
    C = coef.c.y / coef.lastC;
    D = coef.c.z / coef.lastC;

    /*  substitute x = y - A/4 to eliminate cubic term:
    x^4 + px^2 + qx + r = 0 */

    sq_A = A * A;
    p = -3.0 / 8 * sq_A + B;
    q = 1.0 / 8 * sq_A * A - 1.0 / 2 * A * B + C;
    r = -3.0 / 256 * sq_A * sq_A + 1.0 / 16 * sq_A * B - 1.0 / 4 * A * C + D;

    if (IsZero(r))
    {
        /* no absolute term: y(y^3 + py + q) = 0 */

        coeffs.x = q;
        coeffs.y = p;
        coeffs.z = 0;
        coeffs.w = 1;

        num = SolveCubic(coeffs, coef.s);
        coef.s.num = 0;
        //s[num++] = 0;
    }
    else
    {
        /* solve the resolvent cubic ... */

        coeffs.x = 1.0 / 2 * r * p - 1.0 / 8 * q * q;
        coeffs.y = -r;
        coeffs.z = -1.0 / 2 * p;
        coeffs.w = 1;

        SolveCubic(coeffs, s);

        /* ... and take the one real solution ... */

        z = coef.s.x;

        /* ... to build two quadric equations */

        u = z * z - r;
        v = 2 * z - p;

        if (IsZero(u))
            u = 0;
        else if (u > 0)
            u = sqrt(u);
        else
            return 0;

        if (IsZero(v))
            v = 0;
        else if (v > 0)
            v = sqrt(v);
        else
            return 0;

        coeffs.x = z - u;
        coeffs.y = q < 0 ? -v : v;
        coeffs.z = 1;

        num = SolveQuadric(coeffs, coef.s);

        coeffs.x = z + u;
        coeffs.y = q < 0 ? v : -v;
        coeffs.z = 1;

        coef.s += num;
        num += SolveQuadric(coeffs, coef.s);
    }

    /* resubstitute */

    sub = 1.0 / 4 * A;

    //for (i = 0; i < num; ++i)
      //  s[i] -= sub;
    coef.s -= sub;

    return num;
}
