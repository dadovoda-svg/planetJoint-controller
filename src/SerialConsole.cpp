#include "SerialConsole.h"

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <esp_system.h>

extern bool toggleTrace();
extern bool setTraceEnabled(bool enabled);
extern bool setTraceMode(uint8_t mode);
extern uint8_t getTraceMode();
extern const char* getTraceModeName();
extern bool toggleTest(float microstepsPerSecond);
extern bool moveStep(float steps);
extern bool calibrateStepsPerDegree(float targetDegrees);
extern float getStdegCalibrationDefaultTargetDeg();
extern void setZero();
extern bool moveJointToDeg(float targetDeg);
extern bool jointMoveTo(float targetDeg, float vmaxDegS, float amaxDegS2);
extern bool jointMoveTo(float targetDeg, float vmaxDegS, float amaxDegS2, float sCurveTimeS);
extern bool jointStop();
extern void stopMotion();
extern void printServoStatus();

const SerialConsole::Command SerialConsole::_commands[] = {
  {
    "help",
    "help",
    "Show available commands",
    &SerialConsole::cmdHelp
  },
  {
    "get",
    "get <key>",
    "Read one parameter value",
    &SerialConsole::cmdGet
  },
  {
    "set",
    "set <key> <value>",
    "Set one existing parameter value in RAM",
    &SerialConsole::cmdSet
  },
  {
    "load",
    "load",
    "Reload parameters from non-volatile memory",
    &SerialConsole::cmdLoad
  },
  {
    "save",
    "save",
    "Save all parameters to non-volatile memory",
    &SerialConsole::cmdSave
  },
  {
    "export",
    "export",
    "Print all key-value pairs",
    &SerialConsole::cmdExport
  },
  {
    "import",
    "import",
    "Enter non-blocking import mode",
    &SerialConsole::cmdImport
  },
  {
    "cancel",
    "cancel",
    "Exit import mode",
    &SerialConsole::cmdCancel
  },
  {
    "zero",
    "zero",
    "Set current encoder position as zero",
    &SerialConsole::cmdZero
  },
  {
    "reboot",
    "reboot",
    "Forza reboot completo del micro",
    &SerialConsole::cmdReboot
  },
  {
    "trace",
    "trace [0..4|on|off]",
    "abilita/disabilita trace o seleziona il set di valori esportati",
    &SerialConsole::cmdTrace
  },
  {
    "go",
    "go <deg/s>",
    "abilita/disabilita test velocità motore in gradi/sec giunto",
    &SerialConsole::cmdGo
  },
  {
    "step",
    "step <steps>",
    "abilita/disabilita test step motore <steps>",
    &SerialConsole::cmdStep
  },
  {
    "calstdeg",
    "calstdeg [deg]",
    "stima stdeg muovendo il giunto e misurando l'encoder",
    &SerialConsole::cmdCalStdeg
  },
  {
    "pos",
    "pos <deg>",
    "muove il giunto alla posizione indicata, relativa allo zero",
    &SerialConsole::cmdPos
  },
  {
    "move",
    "move <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]",
    "planner-style move con target, velocita e accelerazione espliciti",
    &SerialConsole::cmdMove
  },
  {
    "stop",
    "stop",
    "ferma immediatamente ogni movimento",
    &SerialConsole::cmdStop
  },
  {
    "servo",
    "servo",
    "stampa stato controller posizione",
    &SerialConsole::cmdServo
  }
};

const size_t SerialConsole::_commandCount =
  sizeof(SerialConsole::_commands) / sizeof(SerialConsole::_commands[0]);


SerialConsole::SerialConsole(Stream& serial, PersistentParams& params)
  : _serial(serial),
    _params(params)
{
  _rxBuffer[0] = '\0';
}


void SerialConsole::begin(const char* prompt)
{
  if (prompt != nullptr) {
    _prompt = prompt;
  }

  _serial.println();
  _serial.println("Serial console ready.");
  _serial.println("Type 'help' for commands.");
  printPrompt();
}


void SerialConsole::update()
{
  while (_serial.available() > 0) {
    char c = static_cast<char>(_serial.read());
    processChar(c);
  }
}

void SerialConsole::setParamSetCallback(ParamSetCallback callback)
{
  _paramSetCallback = callback;
}


void SerialConsole::notifyParamSet(const char* key)
{
  if (_paramSetCallback != nullptr) {
    _paramSetCallback(key);
  }
}

void SerialConsole::processChar(char c)
{
  /*
   * Line ending handling:
   *
   * Unix / Linux / modern macOS: LF      '\n'
   * Windows:                    CRLF    "\r\n"
   * classic Mac / some tools:    CR      '\r'
   *
   * Both CR and LF are accepted as line terminators.
   * In case of CRLF, the LF immediately following CR is ignored
   * to avoid processing the same line twice.
   */

  if (c == '\n') {
    if (_lastWasCR) {
      _lastWasCR = false;
      return;
    }

    _rxBuffer[_rxIndex] = '\0';

    if (_rxIndex > 0) {
      _serial.println();
      processLine(_rxBuffer);
    } else {
      _serial.println();
    }

    _rxIndex = 0;
    _rxBuffer[0] = '\0';

    printPrompt();
    return;
  }

  if (c == '\r') {
    _lastWasCR = true;

    _rxBuffer[_rxIndex] = '\0';

    if (_rxIndex > 0) {
      _serial.println();
      processLine(_rxBuffer);
    } else {
      _serial.println();
    }

    _rxIndex = 0;
    _rxBuffer[0] = '\0';

    printPrompt();
    return;
  }

  _lastWasCR = false;

  if (c == '\b' || c == 127) {
    if (_rxIndex > 0) {
      _rxIndex--;
      _rxBuffer[_rxIndex] = '\0';
      _serial.print("\b \b");
    }
    return;
  }

  if (_rxIndex < RX_BUFFER_SIZE - 1) {
    _rxBuffer[_rxIndex++] = c;
    _rxBuffer[_rxIndex] = '\0';

    // Echo locale
    _serial.write(c);
  } else {
    _serial.println();
    _serial.println("ERR line too long");

    _rxIndex = 0;
    _rxBuffer[0] = '\0';
    _lastWasCR = false;

    printPrompt();
  }
}

void SerialConsole::processLine(char* line)
{
  switch (_mode) {
    case Mode::Normal:
      processNormalLine(line);
      break;

    case Mode::Import:
      processImportLine(line);
      break;
  }
}


void SerialConsole::processNormalLine(char* line)
{
  char* argv[MAX_ARGS];
  int argc = tokenize(line, argv, MAX_ARGS);

  if (argc <= 0) {
    return;
  }

  const Command* cmd = findCommand(argv[0]);

  if (cmd == nullptr) {
    _serial.print("ERR unknown command: ");
    _serial.println(argv[0]);
    _serial.println("Type 'help' for commands.");
    return;
  }

  (this->*(cmd->handler))(argc, argv);
}


void SerialConsole::processImportLine(char* line)
{
  if (strcmp(line, ".") == 0) {
    _mode = Mode::Normal;
    _serial.println("OK import completed");
    return;
  }

  if (strcmp(line, "cancel") == 0) {
    _mode = Mode::Normal;
    _serial.println("OK import cancelled");
    return;
  }

  if (line[0] == '\0') {
    return;
  }

  if (line[0] == '#') {
    return;
  }

  char key[PersistentParams::MAX_KEY_LEN + 1];
  float value = 0.0f;

  if (!parseKeyValueLine(line, key, sizeof(key), value)) {
    _serial.print("ERR invalid line: ");
    _serial.println(line);
    return;
  }

  if (!_params.set(key, value)) {
    _serial.print("ERR cannot set key, unknown or invalid: ");
    _serial.println(key);
    return;
  }

  _serial.print("OK ");
  _serial.print(key);
  _serial.print("=");
  _serial.println(value, 6);
}


int SerialConsole::tokenize(char* line, char* argv[], int maxArgs)
{
  int argc = 0;

  char* token = strtok(line, " \t");

  while (token != nullptr && argc < maxArgs) {
    argv[argc++] = token;
    token = strtok(nullptr, " \t");
  }

  return argc;
}


const SerialConsole::Command* SerialConsole::findCommand(const char* name) const
{
  for (size_t i = 0; i < _commandCount; i++) {
    if (strcmp(name, _commands[i].name) == 0) {
      return &_commands[i];
    }
  }

  return nullptr;
}


void SerialConsole::printPrompt()
{
  if (_mode == Mode::Import) {
    _serial.print("import> ");
  } else {
    _serial.print(_prompt);
  }
}


void SerialConsole::cmdHelp(int argc, char* argv[])
{
  (void)argc;
  (void)argv;

  _serial.println("Available commands:");
  _serial.println();

  for (size_t i = 0; i < _commandCount; i++) {
    _serial.print("  ");
    _serial.print(_commands[i].usage);

    size_t len = strlen(_commands[i].usage);
    while (len++ < 26) {
      _serial.print(' ');
    }

    _serial.println(_commands[i].description);
  }

  _serial.println();
}


void SerialConsole::cmdGet(int argc, char* argv[])
{
  if (argc != 2) {
    _serial.println("ERR usage: get <key>");
    return;
  }

  const char* key = argv[1];
  float value = 0.0f;

  if (!_params.get(key, value)) {
    _serial.print("ERR key not found or invalid: ");
    _serial.println(key);
    return;
  }

  _serial.print(key);
  _serial.print("=");
  _serial.println(value, 6);
}


void SerialConsole::cmdSet(int argc, char* argv[])
{
  if (argc != 3) {
    _serial.println("ERR usage: set <key> <value>");
    return;
  }

  const char* key = argv[1];

  char* endPtr = nullptr;
  float value = strtof(argv[2], &endPtr);

  if (endPtr == argv[2] || *endPtr != '\0') {
    _serial.print("ERR invalid value: ");
    _serial.println(argv[2]);
    return;
  }

  if (!_params.set(key, value)) {
    _serial.print("ERR cannot set key, unknown or invalid: ");
    _serial.println(key);
    return;
  }

  _serial.print("OK ");
  _serial.print(key);
  _serial.print("=");
  _serial.println(value, 6);

  notifyParamSet(key);
}


void SerialConsole::cmdLoad(int argc, char* argv[])
{
  if (argc != 1) {
    _serial.println("ERR usage: load");
    return;
  }

  PersistentParams::LoadResult result = _params.loadDetailed();

  if (result != PersistentParams::LoadResult::Ok) {
    _serial.print("ERR load failed: ");
    _serial.println(loadResultToString(result));
    return;
  }

  _serial.println("OK parameters loaded");
}


void SerialConsole::cmdSave(int argc, char* argv[])
{
  if (argc != 1) {
    _serial.println("ERR usage: save");
    return;
  }

  if (!_params.save()) {
    _serial.println("ERR save failed");
    return;
  }

  _serial.println("OK parameters saved");
}


void SerialConsole::cmdExport(int argc, char* argv[])
{
  if (argc != 1) {
    _serial.println("ERR usage: export");
    return;
  }

  _serial.println("# BEGIN PARAM EXPORT");

  uint8_t paramCount = _params.count();

  for (uint8_t i = 0; i < paramCount; i++) {
    const char* key = nullptr;
    float value = 0.0f;

    if (_params.getByIndex(i, key, value)) {
      _serial.print(key);
      _serial.print("=");
      _serial.println(value, 6);
    }
  }

  _serial.println("# END PARAM EXPORT");
}


void SerialConsole::cmdImport(int argc, char* argv[])
{
  if (argc != 1) {
    _serial.println("ERR usage: import");
    return;
  }

  _mode = Mode::Import;

  _serial.println("Import mode.");
  _serial.println("Send lines as:");
  _serial.println("  KEY=value");
  _serial.println("End with a single dot '.' or type 'cancel'.");
}


void SerialConsole::cmdCancel(int argc, char* argv[])
{
  if (argc != 1) {
    _serial.println("ERR usage: cancel");
    return;
  }

  if (_mode == Mode::Import) {
    _mode = Mode::Normal;
    _serial.println("OK import cancelled");
  } else {
    _serial.println("OK nothing to cancel");
  }
}

void SerialConsole::cmdZero(int argc, char* argv[])
{
  (void)argv;

  if (argc != 1) {
    _serial.println("ERR usage: zero");
    return;
  }

  _serial.println("OK setting zero position...");
  _serial.flush();

  setZero();
}

void SerialConsole::cmdReboot(int argc, char* argv[])
{
  (void)argv;

  if (argc != 1) {
    _serial.println("ERR usage: reboot");
    return;
  }

  _serial.println("OK rebooting MCU...");
  _serial.flush();

  delay(100);

  ESP.restart();
}

void SerialConsole::cmdTrace(int argc, char* argv[])
{
  if (argc == 1) {
    const bool enabled = toggleTrace();

    _serial.print("OK trace ");
    _serial.print(enabled ? "enabled" : "disabled");
    _serial.print(" mode=");
    _serial.print(getTraceMode());
    _serial.print(" ");
    _serial.println(getTraceModeName());
    return;
  }

  if (argc != 2) {
    _serial.println("ERR usage: trace [0..4|on|off]");
    return;
  }

  if (strcmp(argv[1], "on") == 0) {
    setTraceEnabled(true);
    _serial.print("OK trace enabled mode=");
    _serial.print(getTraceMode());
    _serial.print(" ");
    _serial.println(getTraceModeName());
    return;
  }

  if (strcmp(argv[1], "off") == 0) {
    setTraceEnabled(false);
    _serial.println("OK trace disabled");
    return;
  }

  char* endPtr = nullptr;
  const long mode = strtol(argv[1], &endPtr, 10);

  if (endPtr == argv[1] || *endPtr != '\0' || mode < 0 || mode > 4) {
    _serial.println("ERR usage: trace [0..4|on|off]");
    _serial.println("  0 FULL");
    _serial.println("  1 POS_TARGET");
    _serial.println("  2 POS_TARGET_VEL");
    _serial.println("  3 PID");
    _serial.println("  4 SCURVE");
    return;
  }

  if (!setTraceMode(static_cast<uint8_t>(mode))) {
    _serial.println("ERR invalid trace mode");
    return;
  }

  _serial.print("OK trace enabled mode=");
  _serial.print(getTraceMode());
  _serial.print(" ");
  _serial.println(getTraceModeName());
}

void SerialConsole::cmdGo(int argc, char* argv[])
{
  (void)argv;

  if (argc != 2) {
    _serial.println("ERR usage: go <deg/s>");
    return;
  }

  char* endPtr = nullptr;
  float value = strtof(argv[1], &endPtr);

  if (endPtr == argv[1] || *endPtr != '\0') {
    _serial.print("ERR invalid value: ");
    _serial.println(argv[1]);
    return;
  }

  bool enabled = toggleTest(value);

  _serial.print("OK velocity test ");
  _serial.println(enabled ? "enabled" : "disabled");
}

void SerialConsole::cmdStep(int argc, char* argv[])
{
  (void)argv;

  if (argc != 2) {
    _serial.println("ERR usage: step <steps>");
    return;
  }

  char* endPtr = nullptr;
  float value = strtof(argv[1], &endPtr);

  if (endPtr == argv[1] || *endPtr != '\0') {
    _serial.print("ERR invalid value: ");
    _serial.println(argv[1]);
    return;
  }

  bool enabled = moveStep(value);

  _serial.print("OK test Step ");
  _serial.println(enabled ? "enabled" : "disabled");
}



void SerialConsole::cmdCalStdeg(int argc, char* argv[])
{
  float targetDeg = getStdegCalibrationDefaultTargetDeg();

  if (argc > 2) {
    _serial.println("ERR usage: calstdeg [deg]");
    return;
  }

  if (argc == 2) {
    char* endPtr = nullptr;
    targetDeg = strtof(argv[1], &endPtr);

    if (endPtr == argv[1] || *endPtr != '\0') {
      _serial.print("ERR invalid value: ");
      _serial.println(argv[1]);
      return;
    }
  }

  bool ok = calibrateStepsPerDegree(targetDeg);
  _serial.print("OK calstdeg ");
  _serial.println(ok ? "completed" : "failed");
}

void SerialConsole::cmdPos(int argc, char* argv[])
{
  if (argc != 2) {
    _serial.println("ERR usage: pos <deg>");
    return;
  }

  char* endPtr = nullptr;
  float value = strtof(argv[1], &endPtr);

  if (endPtr == argv[1] || *endPtr != '\0') {
    _serial.print("ERR invalid value: ");
    _serial.println(argv[1]);
    return;
  }

  if (moveJointToDeg(value)) {
    _serial.print("OK moving to ");
    _serial.print(value, 3);
    _serial.println(" deg");
  } else {
    _serial.println("ERR move rejected");
  }
}

void SerialConsole::cmdMove(int argc, char* argv[])
{
  if (argc != 4 && argc != 5) {
    _serial.println("ERR usage: move <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]");
    return;
  }

  char* endPtr = nullptr;

  const float target = strtof(argv[1], &endPtr);
  if (endPtr == argv[1] || *endPtr != '\0') {
    _serial.print("ERR invalid target_deg: ");
    _serial.println(argv[1]);
    return;
  }

  const float vmax = strtof(argv[2], &endPtr);
  if (endPtr == argv[2] || *endPtr != '\0') {
    _serial.print("ERR invalid vmax_deg_s: ");
    _serial.println(argv[2]);
    return;
  }

  const float amax = strtof(argv[3], &endPtr);
  if (endPtr == argv[3] || *endPtr != '\0') {
    _serial.print("ERR invalid amax_deg_s2: ");
    _serial.println(argv[3]);
    return;
  }

  bool ok = false;

  if (argc == 5) {
    const float sct = strtof(argv[4], &endPtr);
    if (endPtr == argv[4] || *endPtr != '\0') {
      _serial.print("ERR invalid sct_s: ");
      _serial.println(argv[4]);
      return;
    }

    ok = jointMoveTo(target, vmax, amax, sct);
  } else {
    ok = jointMoveTo(target, vmax, amax);
  }

  _serial.println(ok ? "OK move accepted" : "ERR move rejected");
}

void SerialConsole::cmdStop(int argc, char* argv[])
{
  (void)argv;

  if (argc != 1) {
    _serial.println("ERR usage: stop");
    return;
  }

  jointStop();
  _serial.println("OK stopped");
}

void SerialConsole::cmdServo(int argc, char* argv[])
{
  (void)argv;

  if (argc != 1) {
    _serial.println("ERR usage: servo");
    return;
  }

  printServoStatus();
}

bool SerialConsole::parseKeyValueLine(
  const char* line,
  char* keyOut,
  size_t keySize,
  float& valueOut
)
{
  const char* eq = strchr(line, '=');

  if (eq == nullptr) {
    return false;
  }

  size_t keyLen = static_cast<size_t>(eq - line);

  if (keyLen == 0) {
    return false;
  }

  if (keyLen > PersistentParams::MAX_KEY_LEN) {
    return false;
  }

  if (keyLen >= keySize) {
    return false;
  }

  memcpy(keyOut, line, keyLen);
  keyOut[keyLen] = '\0';

  for (size_t i = 0; i < keyLen; i++) {
    if (!isalnum(static_cast<unsigned char>(keyOut[i]))) {
      return false;
    }
  }

  const char* valueStr = eq + 1;

  if (*valueStr == '\0') {
    return false;
  }

  char* endPtr = nullptr;
  float value = strtof(valueStr, &endPtr);

  if (endPtr == valueStr) {
    return false;
  }

  while (*endPtr == ' ' || *endPtr == '\t') {
    endPtr++;
  }

  if (*endPtr != '\0') {
    return false;
  }

  valueOut = value;
  return true;
}


const char* SerialConsole::loadResultToString(PersistentParams::LoadResult result) const
{
  switch (result) {
    case PersistentParams::LoadResult::Ok:
      return "Ok";

    case PersistentParams::LoadResult::NotFound:
      return "NotFound";

    case PersistentParams::LoadResult::InvalidData:
      return "InvalidData";

    case PersistentParams::LoadResult::CrcError:
      return "CrcError";

    case PersistentParams::LoadResult::TooManyParams:
      return "TooManyParams";

    default:
      return "Unknown";
  }
}
