using Unity.Mathematics;

namespace Physics
{
    public static class PhysicsMath
    {
        private const float DEFAULT_EPSILON = 1e-4f;

        public static bool Approximately(float a, float b, float epsilon = DEFAULT_EPSILON)
        {
            return math.abs(a - b) < epsilon;
        }

        public static bool Approximately(float3 a, float3 b, float epsilon = DEFAULT_EPSILON)
        {
            return math.lengthsq(a - b) < epsilon * epsilon;
        }

        public static bool Approximately(float3[] a, float3[] b, float epsilon = DEFAULT_EPSILON)
        {
            if (a == null || b == null || a.Length != b.Length) return false;

            for (int index = 0, max = a.Length; index < max; ++index)
            {
                if (Approximately(a[index], b[index]) == false) return false;
            }
            return true;
        }
    }
}