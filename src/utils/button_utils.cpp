#ifndef HS_MAIN_COMPOSED
void __hs_utils_button_stub() {}
#else

static void handleButtons() {
  uint32_t now = millis();

  if ((int32_t)(gIgnoreButtonsUntilMs - now) > 0) {
    isrNextFlag = false;
    isrPrevFlag = false;
    return;
  }

  static uint32_t lastBtnN = 0;
  static uint32_t lastBtnP = 0;

  if (isrNextFlag) {
    isrNextFlag = false;
    isrPrevFlag = false;
    if (now - lastBtnN > DEBOUNCE_MS) {
      lastBtnN = now;
      gIgnoreButtonsUntilMs = now + DEBOUNCE_MS;
      page = (page + 1) % NUM_PAGES;
      drawPage();
      return;
    }
  }

  if (isrPrevFlag) {
    isrPrevFlag = false;
    isrNextFlag = false;
    if (now - lastBtnP > DEBOUNCE_MS) {
      lastBtnP = now;
      gIgnoreButtonsUntilMs = now + DEBOUNCE_MS;
      page = (page - 1 + NUM_PAGES) % NUM_PAGES;
      drawPage();
      return;
    }
  }
}

#endif
