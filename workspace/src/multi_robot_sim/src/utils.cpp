//Description: This file contains the implementation of custom functions.
#include "utils.h"

void killTerminal() {
  #if defined(__unix__) || defined(__APPLE__)
    cout << "Closing the terminal" << endl;
    sleep(1);
    system("kill -9 $(ps -p $PPID -o ppid=)");
  #else
    #error "Unknown system"
  #endif
}

void clearTerminal() {
  // Move the cursor to the start position for the messages
  cout << "\033[" << 12 << ";1H";
  // Clear the line
  cout << "\033[K";
}

Json::Value readJson(const string& in_path) {
    cout << "Configuration file path: " << in_path << endl;

    ifstream file(in_path, ifstream::binary);
    if (!file) {
        throw runtime_error("Could not open the file: " + in_path);
    }

    Json::Value root;
    Json::CharReaderBuilder readerBuilder;
    string errs;

    if (!Json::parseFromStream(readerBuilder, file, &root, &errs)) {
        throw runtime_error("Failed to parse JSON file: " + errs);
    }

    return root;
}
