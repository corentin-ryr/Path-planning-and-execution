using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Analysis
{
    public class MathHelper
    {

        enum CurveState
        {
            GoingDown = 0, EqGoingDown = 1, NotGoingDown = 2
        }

        /// <summary>
        /// Returns the indices of the local minimas
        /// </summary>
        /// <param name="a">List of float that represent the function</param>
        /// <returns></returns>
        public static IList<int> GetValleys(IList<float> a)
        {
            var res = new List<int>();
            if (a.Count < 2)
            {
                return res;
            }
            int lastEq = 0;
            CurveState s = CurveState.NotGoingDown;
            for (var i = 1; i != a.Count; i++)
            {
                switch (Mathf.Sign(a[i] - a[i - 1]))
                {
                    case -1:
                        s = CurveState.GoingDown;
                        break;
                    case 0:
                        if (s == CurveState.GoingDown)
                        {
                            lastEq = i;
                        }
                        s = (s == CurveState.NotGoingDown)
                          ? CurveState.NotGoingDown
                          : CurveState.EqGoingDown;
                        break;
                    case 1:
                        if (s == CurveState.GoingDown)
                        {
                            res.Add(i - 1);
                        }
                        else if (s == CurveState.EqGoingDown)
                        {
                            res.Add((lastEq + i - 1) / 2);
                        }
                        s = CurveState.NotGoingDown;
                        break;
                }
            }
            return res;
        }

        /// <summary>
        ///     Returns the function with the greatest integral given 2 constraints: a function to stay under and a slope constraint.
        /// </summary>
        /// <param name="a">The constraint function (the returned function cannot go above a)</param>
        /// <param name="AccelerationAtSpeed">Slope constraint</param>
        /// <returns></returns>
        public static IList<float> BestFunctionWithSlopeConstraints(IList<float> a, Func<float, float> AccelerationAtSpeed) {
            //TODO
            return a;
        }
    }
}