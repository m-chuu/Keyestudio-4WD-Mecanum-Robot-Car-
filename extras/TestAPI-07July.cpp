// ================================================================
// TestAPI.cpp - Desktop test: fetch latest Favoriot stream entry
// ================================================================
// GETs the newest data stream for DEVICE_DEVELOPER_ID and prints
// its data.distance value (e.g. "TaskMiddle_complete").
//
// Build & run (macOS):
//   g++ -std=c++17 extras/TestAPI.cpp -o /tmp/TestAPI \
//       -I/usr/local/include -lcurl && /tmp/TestAPI
// ================================================================

#include <iostream>
#include <string>
#include <ctime>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include "../secrets.h"

using json = nlohmann::json;

// Callback to capture HTTP response
size_t writeCallback(void* contents, size_t size, size_t nmemb, std::string* s) {
  s->append((char*)contents, size * nmemb);
  return size * nmemb;
}

// URL-encode the device developer id (the '@' becomes %40)
std::string urlEncode(CURL* curl, const std::string& value) {
  char* encoded = curl_easy_escape(curl, value.c_str(), (int)value.length());
  std::string result(encoded);
  curl_free(encoded);
  return result;
}

int main() {
  std::cout << "\n========================================\n";
  std::cout << "Favoriot API Test - Latest Stream Entry\n";
  std::cout << "========================================\n\n";

  CURL* curl = curl_easy_init();
  if (!curl) {
    std::cerr << "[Favoriot] ✗ Failed to initialize CURL\n";
    return 1;
  }

  // max=1 -> only the newest entry (results are sorted newest first)
  std::string url = std::string(FAVORIOT_API_URL) +
                    "?device_developer_id=" + urlEncode(curl, DEVICE_DEVELOPER_ID) +
                    "&max=1";

  std::cout << "[Config] Device ID: " << DEVICE_DEVELOPER_ID << "\n";
  std::cout << "[Config] URL: " << url << "\n\n";

  std::string responseData;
  struct curl_slist* headers = nullptr;
  headers = curl_slist_append(headers, "accept: application/json");
  std::string apiKeyHeader = "apikey: " + std::string(FAVORIOT_JWT_TOKEN);
  headers = curl_slist_append(headers, apiKeyHeader.c_str());

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &responseData);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);

  CURLcode res = curl_easy_perform(curl);

  int exitCode = 0;
  if (res != CURLE_OK) {
    std::cerr << "[Favoriot] ✗ CURL Error: " << curl_easy_strerror(res) << "\n";
    exitCode = 1;
  } else {
    long httpCode = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);
    std::cout << "[Favoriot] HTTP Response Code: " << httpCode << "\n";

    if (httpCode != 200) {
      std::cerr << "[Favoriot] ✗ Server returned error code: " << httpCode << "\n";
      std::cerr << "[Favoriot] Response: " << responseData << "\n";
      exitCode = 1;
    } else {
      try {
        json response = json::parse(responseData);
        int numFound = response.value("numFound", 0);
        std::cout << "[Favoriot] Total entries on server: " << numFound << "\n";

        if (response["results"].empty()) {
          std::cout << "[Favoriot] ✗ No stream entries found for this device\n";
          exitCode = 1;
        } else {
          const json& latest = response["results"][0];
          std::string distance = latest["data"].value("distance", "(missing)");
          std::string createdAt = latest.value("stream_created_at", "(unknown)");

          std::cout << "\n---------- Latest Entry ----------\n";
          std::cout << "Timestamp : " << createdAt << "\n";
          std::cout << "Distance  : " << distance << "\n";
          std::cout << "----------------------------------\n";

          if (distance == "TaskMiddle_complete") {
            std::cout << "[Favoriot] ✓ Latest data is TaskMiddle_complete\n";
          } else {
            std::cout << "[Favoriot] ✗ Latest data is NOT TaskMiddle_complete\n";
            exitCode = 1;
          }
        }
      } catch (const json::exception& e) {
        std::cerr << "[Favoriot] ✗ JSON parse error: " << e.what() << "\n";
        std::cerr << "[Favoriot] Response: " << responseData << "\n";
        exitCode = 1;
      }
    }
  }

  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  std::cout << "\n========================================\n";
  std::cout << "Test Complete\n";
  std::cout << "========================================\n";
  return exitCode;
}
