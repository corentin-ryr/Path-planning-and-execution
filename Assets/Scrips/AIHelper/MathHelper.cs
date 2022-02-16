using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

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
        public static List<int> GetValleys(List<double> a, bool maximums = false)
        {
            float reverse = maximums ? -1f : 1f;

            var res = new List<int>();
            if (a.Count < 2)
            {
                return res;
            }
            int lastEq = 0;
            CurveState s = CurveState.NotGoingDown;
            for (var i = 1; i != a.Count; i++)
            {
                switch (Math.Sign(reverse * a[i] - reverse * a[i - 1]))
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
        public static List<double> BestFunctionWithSlopeConstraints(List<double[]> xyFunction, Func<double, double> AccelerationAtSpeed, Func<double, double> DeccelerationAtSpeed)
        {
            List<double> a = new List<double>();
            for (int i = 0; i < xyFunction.Count; i++)
            {
                a.Add(xyFunction[i][1]);
            }
            List<int> valleys = GetValleys(a);
            valleys.Add(0);
            double[] b = new double[xyFunction.Count];
            for (int i = 0; i < b.Length; i++)
            {
                b[i] = float.MaxValue;
            }


            foreach (int valleyIndex in valleys)
            {
                //We compute the partial best function
                double[] tempB = new double[xyFunction.Count];
                tempB[valleyIndex] = xyFunction[valleyIndex][1];
                b[valleyIndex] = Math.Min(tempB[valleyIndex], b[valleyIndex]);

                //First go through the left side of the function
                bool canContinue = true;
                int i = valleyIndex;
                while (canContinue && i > 0)
                {
                    if (xyFunction[i - 1][1] >= xyFunction[i][1] || tempB[i] + Math.Abs(xyFunction[i][0] - xyFunction[i - 1][0]) * DeccelerationAtSpeed(tempB[i]) <= xyFunction[i - 1][1])
                    {
                        tempB[i - 1] = Math.Min(tempB[i] + Math.Abs(xyFunction[i][0] - xyFunction[i - 1][0]) * DeccelerationAtSpeed(tempB[i]), xyFunction[i - 1][1]);
                        b[i - 1] = Math.Min(tempB[i - 1], b[i - 1]);
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
                    if (xyFunction[i + 1][1] >= xyFunction[i][1] || tempB[i] + Math.Abs(xyFunction[i][0] - xyFunction[i + 1][0]) * AccelerationAtSpeed(tempB[i]) <= xyFunction[i + 1][1])
                    {
                        tempB[i + 1] = Math.Min(tempB[i] + Math.Abs(xyFunction[i][0] - xyFunction[i + 1][0]) * AccelerationAtSpeed(tempB[i]), xyFunction[i + 1][1]);
                        b[i + 1] = Math.Min(tempB[i + 1], b[i + 1]);
                        i = i + 1;
                    }
                    else
                    {
                        canContinue = false;
                    }
                }

            }
            return new List<double>(b);
        }

        public static int DichotomicSearch(float[] inputArray, float key)
        {
            int min = 0;
            int max = inputArray.Length - 1;
            while (max - min > 1)
            {
                int mid = (min + max) / 2;
                if (key == inputArray[mid])
                {
                    return mid;
                }
                else if (key < inputArray[mid])
                {
                    max = mid;
                }
                else
                {
                    min = mid;
                }
            }
            return min;
        }

        public static List<double[]> pieceWiseConstantFromSpeedProfile(List<double[]> xyFunction)
        {
            //TODO

            List<double> speedProfile = new List<double>();
            for (int i = 0; i < xyFunction.Count; i++)
            {
                speedProfile.Add(xyFunction[i][1]);
            }
            //Get the local minimas
            List<int> minimaIndices = GetValleys(speedProfile, false);

            //Get the local maximas
            List<int> maximaIndices = GetValleys(speedProfile, true);
            maximaIndices.Add(xyFunction.Count - 1);

            //For each extremum, store its distance to the start and the target speed BEFORE it (in the segment between the previous extrema and this one)
            //That way we can find the target speed by going through the list and finding the first element with a distance bigger than the current distance.
            List<double[]> targetSpeedAtDistance = new List<double[]>();
            foreach (int index in minimaIndices)
            {
                targetSpeedAtDistance.Add(new double[] { xyFunction[index][0], xyFunction[index][3] });
            }
            foreach (int index in maximaIndices)
            {
                targetSpeedAtDistance.Add(new double[] { xyFunction[index][0], xyFunction[index][3] });
            }

            List<double[]> SortedList = targetSpeedAtDistance.OrderBy(o=>o[0]).ToList();
            return SortedList;
        }
    }
}