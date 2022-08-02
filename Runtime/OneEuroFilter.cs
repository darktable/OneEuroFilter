using System;
using System.Runtime.CompilerServices;
using UnityEngine;

public interface IOneEuroFilter
{
    float Beta { get; }
    float MinCutoff { get; }
}

[Serializable]
public struct OneEuroFilterParams : IOneEuroFilter
{
    [Range(0.0f, 2.0f)]
    public float beta;

    [Range(0.0f, 10.0f)]
    public float minCutoff;

    public OneEuroFilterParams(float pBeta, float pMinCutoff)
    {
        beta = pBeta;
        minCutoff = pMinCutoff;
    }

    public float Beta
    {
        get => beta;
    }

    public float MinCutoff
    {
        get => minCutoff;
    }
}

public sealed class OneEuroFilter : OneEuroFilterBase<float>
{
    public OneEuroFilter(float pBeta = 0.0f, float pMinCutoff = 1.0f)
        : base(pBeta, pMinCutoff) { }

    public override void Reset()
    {
        SetPrevious(0, 0, 0);
    }

    #region Public step function

    public override float Step(float pT, float pX)
    {
        float tE = pT - m_PrevT;

        // Do nothing if the time difference is too small.
        if (tE < k_MinTimeDelta)
        {
            return m_PrevX;
        }

        float dx = (pX - m_PrevX) / tE;
        float dxRes = Mathf.Lerp(m_PrevDx, dx, Alpha(tE, k_DCutOff));

        float cutoff = m_MinCutoff + m_Beta * dxRes;
        float xRes = Mathf.Lerp(m_PrevX, pX, Alpha(tE, cutoff));

        SetPrevious(pT, xRes, dxRes);

        return xRes;
    }

    #endregion
}

public sealed class OneEuroFilter2 : OneEuroFilterBase<Vector2>
{
    public OneEuroFilter2(float pBeta = 0.0f, float pMinCutoff = 1.0f)
        : base(pBeta, pMinCutoff) { }

    public override void Reset()
    {
        SetPrevious(0, Vector2.zero, Vector2.zero);
    }

    #region Public step function

    public override Vector2 Step(float pT, Vector2 pX)
    {
        float tE = pT - m_PrevT;

        // Do nothing if the time difference is too small.
        if (tE < k_MinTimeDelta)
        {
            return m_PrevX;
        }

        var dx = new Vector2((pX.x - m_PrevX.x) / tE, (pX.y - m_PrevX.y) / tE);
        var dxRes = Vector2.Lerp(m_PrevDx, dx, Alpha(tE, k_DCutOff));

        float cutoff = m_MinCutoff + m_Beta * dxRes.magnitude;
        var xRes = Vector2.Lerp(m_PrevX, pX, Alpha(tE, cutoff));

        SetPrevious(pT, xRes, dxRes);

        return xRes;
    }

    #endregion
}

public sealed class OneEuroFilter3 : OneEuroFilterBase<Vector3>
{
    public OneEuroFilter3(float pBeta = 0.0f, float pMinCutoff = 1.0f)
        : base(pBeta, pMinCutoff) { }

    public override void Reset()
    {
        SetPrevious(0, Vector3.zero, Vector3.zero);
    }

    #region Public step function

    public override Vector3 Step(float pT, Vector3 pX)
    {
        float tE = pT - m_PrevT;

        // Do nothing if the time difference is too small.
        if (tE < k_MinTimeDelta)
        {
            return m_PrevX;
        }

        var dx = new Vector3((pX.x - m_PrevX.x) / tE, (pX.y - m_PrevX.y) / tE, (pX.z - m_PrevX.z) / tE);
        var dxRes = Vector3.Lerp(m_PrevDx, dx, Alpha(tE, k_DCutOff));

        float cutoff = m_MinCutoff + m_Beta * dxRes.magnitude;
        var xRes = Vector3.Lerp(m_PrevX, pX, Alpha(tE, cutoff));

        SetPrevious(pT, xRes, dxRes);

        return xRes;
    }

    #endregion
}

public sealed class OneEuroFilterQuaternion : OneEuroFilterBase<Quaternion>
{
    public OneEuroFilterQuaternion(float pBeta = 0.0f, float pMinCutoff = 1.0f)
        : base(pBeta, pMinCutoff)
    {
        Reset();
    }

    public override void Reset()
    {
        SetPrevious(0, Quaternion.identity, Quaternion.identity);
    }

    #region Public step function

    public override Quaternion Step(float pT, Quaternion pX)
    {
        float tE = pT - m_PrevT;

        // Do nothing if the time difference is too small.
        if (tE < k_MinTimeDelta)
        {
            return m_PrevX;
        }

        // derived from the VRPN OneEuro Quaternion filter
        float rate = 1.0f / tE;

        // Quaternion subtraction is multiplication by the inverse
        var dx = pX * Quaternion.Inverse(m_PrevX);

        dx.Set(dx.x * rate, dx.y * rate, dx.z * rate, dx.w * rate);

        dx.w += 1.0f - rate;
        dx = Quaternion.Normalize(dx);

        float cutoff = m_MinCutoff + m_Beta * 2.0f * Mathf.Acos(dx.w);
        var xRes = Quaternion.Slerp(m_PrevX, pX, Alpha(tE, cutoff));

        SetPrevious(pT, xRes, dx);

        return xRes;
    }

    #endregion
}

public abstract class OneEuroFilterBase<T> : IOneEuroFilter where T : IEquatable<T>
{
    #region Public properties

    protected float m_Beta;
    protected float m_MinCutoff;

    public float Beta
    {
        get => m_Beta;
    }

    public float MinCutoff
    {
        get => m_MinCutoff;
    }

    #endregion

    public void SetParams(OneEuroFilterParams pParams)
    {
        SetParams(pParams.beta, pParams.minCutoff);
    }

    public void SetParams(float pBeta, float pMinCutoff)
    {
        m_Beta = pBeta;
        m_MinCutoff = pMinCutoff;
    }

    public abstract void Reset();

    #region Public step function

    public abstract T Step(float pT, T pX);

    #endregion

    protected OneEuroFilterBase(float pBeta, float pMinCutoff)
    {
        m_Beta = pBeta;
        m_MinCutoff = pMinCutoff;
    }

    protected OneEuroFilterBase() { }

    #region Protected class members

    protected const float k_DCutOff = 1.0f;
    protected const float k_MinTimeDelta = 1e-5f;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected static float Alpha(float pTe, float pCutoff)
    {
        float r = 2 * Mathf.PI * pCutoff * pTe;
        return r / (r + 1);
    }

    #endregion

    #region Previous state variables as a tuple

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected void SetPrevious(float pT, T pX, T pDx)
    {
        m_PrevT = pT;
        m_PrevX = pX;
        m_PrevDx = pDx;
    }

    protected float m_PrevT;
    protected T m_PrevX;
    protected T m_PrevDx;

    #endregion
}
