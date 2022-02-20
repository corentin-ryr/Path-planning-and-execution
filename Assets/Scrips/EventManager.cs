using UnityEngine.Events;

public static class EventManager 
{
    public static event UnityAction FinishedOptimizing;
    public static void OnFinishedOptimizing() => FinishedOptimizing?.Invoke();
}
