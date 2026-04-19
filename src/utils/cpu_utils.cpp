#ifndef HS_MAIN_COMPOSED
void __hs_utils_cpu_stub() {}
#else

static float measureCpuLoadPct() {
  static bool cpuLoadFilteredSeeded = false;
  static float cpuLoadFiltered = 0.0f;

  uint32_t nowMs = millis();
  if (cpuLastLoadSampleMs == 0) {
    cpuLastLoadSampleMs = nowMs;
    cpuPrevIdleHookCount = cpuIdleHookCount;
  }

  uint32_t elapsedMs = nowMs - cpuLastLoadSampleMs;
  if (elapsedMs < 1000U) {
    return gCpuLoad;
  }

  float loadOut = 0.0f;
  bool measured = false;
  if (getCpuLoadFromRuntimeStats(loadOut)) {
    measured = true;
  } else {
    uint32_t idleNow = cpuIdleHookCount;
    uint32_t idleDelta = idleNow - cpuPrevIdleHookCount;
    cpuPrevIdleHookCount = idleNow;

    float idleRate = (elapsedMs > 0U) ? ((float)idleDelta / (float)elapsedMs) : 0.0f;

    if (cpuIdleRateRef < 0.001f) {
      cpuIdleRateRef = idleRate;
      loadOut = 0.0f;
      measured = true;
    } else {
      if (idleRate > cpuIdleRateRef) {
        cpuIdleRateRef = idleRate;
      } else {
        cpuIdleRateRef = cpuIdleRateRef * 0.999f + idleRate * 0.001f;
      }

      if (cpuIdleRateRef < 0.001f) {
        loadOut = 0.0f;
      } else {
        float idlePct = (100.0f * idleRate) / cpuIdleRateRef;
        loadOut = constrain(100.0f - idlePct, 0.0f, 100.0f);
      }
      measured = true;
    }
  }

  cpuLastLoadSampleMs = nowMs;
  if (measured) {
    float targetLoad = constrain(loadOut, 0.0f, 100.0f);
    if (!cpuLoadFilteredSeeded) {
      cpuLoadFiltered = targetLoad;
      cpuLoadFilteredSeeded = true;
    } else {
      float alpha = (targetLoad > cpuLoadFiltered) ? 0.35f : 0.20f;
      float next = cpuLoadFiltered + alpha * (targetLoad - cpuLoadFiltered);

      float maxStepPerSample = 3.0f;
      float step = next - cpuLoadFiltered;
      if (step > maxStepPerSample) step = maxStepPerSample;
      if (step < -maxStepPerSample) step = -maxStepPerSample;
      cpuLoadFiltered += step;
    }

    if (cpuLoadFiltered < 0.3f) cpuLoadFiltered = 0.0f;
    return constrain(cpuLoadFiltered, 0.0f, 100.0f);
  }
  return gCpuLoad;
}

static bool cpuIdleHook() {
  cpuIdleHookCount++;
  return true;
}

static bool getCpuLoadFromRuntimeStats(float& loadOut) {
#if (configUSE_TRACE_FACILITY == 1) && (configGENERATE_RUN_TIME_STATS == 1)
  UBaseType_t taskCount = uxTaskGetNumberOfTasks();
  if (taskCount == 0) return false;

  TaskStatus_t* stats = (TaskStatus_t*)pvPortMalloc(taskCount * sizeof(TaskStatus_t));
  if (stats == nullptr) return false;

  uint32_t totalRunTime = 0;
  UBaseType_t filled = uxTaskGetSystemState(stats, taskCount, &totalRunTime);
  uint32_t idleRunTime = 0;
  for (UBaseType_t i = 0; i < filled; i++) {
    if (strncmp(stats[i].pcTaskName, "IDLE", 4) == 0) {
      idleRunTime += stats[i].ulRunTimeCounter;
    }
  }
  vPortFree(stats);

  if (!cpuRuntimeSeeded) {
    cpuRuntimeSeeded = true;
    cpuPrevTotalRunTime = totalRunTime;
    cpuPrevIdleRunTime = idleRunTime;
    return false;
  }

  uint32_t dTotal = totalRunTime - cpuPrevTotalRunTime;
  uint32_t dIdle = idleRunTime - cpuPrevIdleRunTime;
  cpuPrevTotalRunTime = totalRunTime;
  cpuPrevIdleRunTime = idleRunTime;

  if (dTotal == 0) return false;

  float idlePct = (100.0f * (float)dIdle) / (float)dTotal;
  loadOut = constrain(100.0f - idlePct, 0.0f, 100.0f);
  return true;
#else
  (void)loadOut;
  return false;
#endif
}

static void cpuStressTask(void* parm) {
  (void)parm;
  volatile float sink = 0.0001f;
  for (;;) {
    if (moduleCpuStress) {
      for (int i = 1; i <= 800; i++) {
        float x = (float)i + sink;
        sink += sqrtf(x) * logf(x + 1.0f);
        sink = fmodf(sink, 1000.0f);
      }
      taskYIELD();
    } else {
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}

#endif
