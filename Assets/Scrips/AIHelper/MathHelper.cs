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
        public static List<int> GetValleys(List<float> a)
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
        /// <param name="a">The constraint function (the returned function cannot go above a). In this case it is the max speed at each point.</param>
        /// <param name="AccelerationAtSpeed">Slope constraint</param>
        /// <returns></returns>
        public static List<float> BestFunctionWithSlopeConstraints(List<float[]> xyFunction, Func<float, float> AccelerationAtSpeed)
        {
            //TODO Check if it works

            List<float> a = new List<float>();
            for (int i = 0; i < xyFunction.Count; i++)
            {
                a.Add(xyFunction[i][1]);
            }
            List<int> valleys = GetValleys(a);
            valleys.Add(0);
            float[] b = new float[xyFunction.Count];
            for (int i = 0; i < b.Length; i++)
            {
                b[i] = float.MaxValue;
            }


            foreach (int valleyIndex in valleys)
            {
                Debug.Log("Valley index: " + valleyIndex);
                //We compute the partial best function
                float[] tempB = new float[xyFunction.Count];
                tempB[valleyIndex] = xyFunction[valleyIndex][1];
                b[valleyIndex] = Mathf.Min(tempB[valleyIndex], b[valleyIndex]);

                //First go through the left side of the function
                bool canContinue = true;
                int i = valleyIndex;
                while (canContinue && i > 0)
                {
                    if (xyFunction[i - 1][1] >= xyFunction[i][1] || tempB[i] + Mathf.Abs(xyFunction[i][0] - xyFunction[i - 1][0]) * AccelerationAtSpeed(tempB[i]) <= xyFunction[i - 1][1])
                    {
                        tempB[i - 1] = Mathf.Min(tempB[i] + Mathf.Abs(xyFunction[i][0] - xyFunction[i - 1][0]) * AccelerationAtSpeed(tempB[i]), xyFunction[i - 1][1]);
                        b[i - 1] = Mathf.Min(tempB[i - 1], b[i - 1]);
                        i = i - 1;
                    }
                    else
                    {
                        canContinue = false;
                    }
                }

                //Then we go through the right side of the function
                canContinue = true;
                i = valleyIndex;
                while (canContinue && i < xyFunction.Count - 1)
                {
                    if (xyFunction[i + 1][1] >= xyFunction[i][1] || tempB[i] + Mathf.Abs(xyFunction[i][0] - xyFunction[i + 1][0]) * AccelerationAtSpeed(tempB[i]) <= xyFunction[i + 1][1])
                    {
                        tempB[i + 1] = Mathf.Min(tempB[i] + Mathf.Abs(xyFunction[i][0] - xyFunction[i + 1][0]) * AccelerationAtSpeed(tempB[i]), xyFunction[i + 1][1]);
                        b[i + 1] = Mathf.Min(tempB[i + 1], b[i + 1]);
                        i = i + 1;
                    }
                    else
                    {
                        canContinue = false;
                    }
                }

            }
            return new List<float>(b);
        }
    }
}