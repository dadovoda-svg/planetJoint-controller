#pragma once

#include <Arduino.h>
#include "params.h"

class SerialConsole {
public:
  typedef void (SerialConsole::*CommandHandler)(int argc, char* argv[]);

  struct Command {
    const char* name;
    const char* usage;
    const char* description;
    CommandHandler handler;
  };

  SerialConsole(Stream& serial, PersistentParams& params);

  void begin(const char* prompt = "> ");
  void update();

private:
  static constexpr size_t RX_BUFFER_SIZE = 128;
  static constexpr size_t MAX_ARGS = 8;
  bool _lastWasCR = false;

  enum class Mode {
    Normal,
    Import
  };

  Stream& _serial;
  PersistentParams& _params;

  const char* _prompt = "> ";
  Mode _mode = Mode::Normal;

  char _rxBuffer[RX_BUFFER_SIZE];
  size_t _rxIndex = 0;

  void processChar(char c);
  void processLine(char* line);

  void processNormalLine(char* line);
  void processImportLine(char* line);

  int tokenize(char* line, char* argv[], int maxArgs);

  const Command* findCommand(const char* name) const;

  void printPrompt();

  void cmdHelp(int argc, char* argv[]);
  void cmdGet(int argc, char* argv[]);
  void cmdSet(int argc, char* argv[]);
  void cmdLoad(int argc, char* argv[]);
  void cmdSave(int argc, char* argv[]);
  void cmdExport(int argc, char* argv[]);
  void cmdImport(int argc, char* argv[]);
  void cmdCancel(int argc, char* argv[]);
  void cmdTrace(int argc, char* argv[]);
  void cmdTest(int argc, char* argv[]);

  bool parseKeyValueLine(
    const char* line,
    char* keyOut,
    size_t keySize,
    float& valueOut
  );

  const char* loadResultToString(PersistentParams::LoadResult result) const;

  static const Command _commands[];
  static const size_t _commandCount;
};
