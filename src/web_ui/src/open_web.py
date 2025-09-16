from selenium import webdriver
from selenium.webdriver.chrome.options import Options
import time

URL = "http://10.20.30.40:8000/map.html"
ORIGIN = "http://10.20.30.40:8000"

def main():
    options = Options()
    options.add_argument("--start-fullscreen")     # open in fullscreen
    options.add_argument("--disable-extensions")
    options.add_argument("--no-default-browser-check")
    options.add_argument("--no-first-run")

    driver = webdriver.Chrome(options=options)

    try:
        # Open a blank page first so CDP commands work cleanly
        driver.get("about:blank")

        # Use Chrome DevTools Protocol to clear cache/cookies/storage
        driver.execute_cdp_cmd("Network.enable", {})
        driver.execute_cdp_cmd("Network.clearBrowserCache", {})
        driver.execute_cdp_cmd("Network.clearBrowserCookies", {})
        driver.execute_cdp_cmd(
            "Storage.clearDataForOrigin",
            {
                "origin": ORIGIN,
                "storageTypes": (
                    "appcache,cache_storage,cookies,local_storage,"
                    "session_storage,service_workers,websql,indexeddb"
                ),
            },
        )

        # Now go to the target page
        driver.get(URL)
        print(f"Opened {URL} in Chrome browser (fullscreen) after clearing cache.")

        # keep open "forever"
        while True:
            time.sleep(3600)

    except KeyboardInterrupt:
        pass
    finally:
        # Close the browser if the script is interrupted
        driver.quit()

if __name__ == "__main__":
    main()
