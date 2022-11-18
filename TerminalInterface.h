#pragma once

#include <cinttypes>
#include <deque>
#include <string>
#include <vector>
// #include <stdio.h>
#include <memory>
#include <iostream> // std::cout, std::right, std::endl
#include <iomanip>  // std::setw
#include <unistd.h>

#define CLI_BUFF_LENGTH 90

namespace VT100
{
    const char HOME[] = "\033[H";
    const char CLEAR_CONSOLE[] = "\033[2J";
    const char CLEAR_TO_END_OF_ROW[] = "\033[0K";

    const char RED[] = "\033[31m";
    const char GREEN[] = "\033[32m";
    const char YELLOW[] = "\033[33m";
    const char BLUE[] = "\033[34m";
    const char MAGENTA[] = "\033[35m";
    const char CYAN[] = "\033[36m";
    const char WHITE[] = "\033[37m";
    const char RESET_COLOR[] = "\033[0m";

    const char BLINKING[] = "\033[5m";
    const char NOT_BLINKING[] = "\033[25m";

    const char HIDE_CURSOR[] = "\033[?25l";
    const char SHOW_CURSOR[] = "\033[?25h";
    const char SAVE_CURSOR[] = "\033 7";
    const char RESTORE_CURSOR[] = "\033 8";

    inline std::string CURSOR_TO_ROW_COL(unsigned row, unsigned col)
    {
        std::stringstream ss;
        ss << "\033[" << row + 1 << ";" << col << "H";
        return ss.str();
    }
    inline std::string CURSOR_TO_ROW(unsigned row)
    {
        std::stringstream ss;
        ss << "\033[" << row + 1 << ";"
           << "0H";
        return ss.str();
    }
    inline std::string CURSOR_TO_COL(int col)
    {
        std::stringstream ss;
        ss << "\033[" << col << "G";
        return ss.str();
    }

    inline std::string SET_WINDOW_SIZE(unsigned rows, unsigned cols)
    {
        std::stringstream ss;
        ss << "\033[8;" << rows << ";" << cols << "H";
        return ss.str();
    }

};

#define TERMINAL_WIDTH 80
#define PRINT_SERVICE_COUNTER 0

namespace TERM
{
    enum
    {
        INFO = 0,
        DEBUG = 1,
        WARNING = 2,
        ERROR = 3
    };

    enum CLI_HEADER_ROWS
    {
        TOP_HEADER,
        MIDDLE_HEADER,
        LOWER_HEADER,
        EMPTY_1,
        NUM_HEADER_ROWS
    };

    const unsigned int MAX_DEBUG_ROWS = 10;
    const unsigned int MAX_CLOCKBUFF_LEN = 64;
}

class TerminalInterface
{
protected:
    uint32_t currentInputCol;
    char rxBuff[CLI_BUFF_LENGTH];
    char *rxPtr;
    void handleCliCommand();
    void resetPrompt();
    std::deque<std::string> debugMessages;

    struct PersistentTerminalField
    {
        uint8_t printRow;
        std::string label;
        bool printAsSexa;
    };
    static bool newInputFlag;

    static void createPipes();

private:
    // std::unique_ptr<std::ostringstream> collector;
    // std::ostringstream* collector;
    uint16_t debugMessageCount;
    uint16_t firstDebugRow = TERM::NUM_HEADER_ROWS + 1;
    uint16_t debugRowOffset = 0;

    uint16_t promptRow;
    uint16_t messageRow;
    std::string ifLabel;
    uint16_t fieldStartCol = 5;
    std::vector<PersistentTerminalField *> persistentFields;

public:
    TerminalInterface(const std::string &);
    virtual ~TerminalInterface();
    static void readInput(); // int read_fd
    // void updateStatusFields(MountControl &);
    void serviceCLI();
    // void addDebugMessage(std::string&, uint8_t);
    void addDebugMessage(const std::string &msg, uint8_t level = TERM::INFO);
    // void clearDebugMessages();
    void printHeader();
    void addPersistentField(const std::string &label, uint8_t printRow);

    // void updatePersistentField(uint8_t printRow, const std::string &fieldValStr);
    template <typename T>
    void updatePersistentField(uint8_t printRow, const T val);
    void printPersistentFieldLabels();

    void inputCallback(std::ios::event ev, std::ios_base &stream, int index);
    // clang-format off

    // clang-format on
};

int fs_sexa(char *out, double a, int w, int fracbase);

template <>
inline void TerminalInterface::updatePersistentField(uint8_t printRow, const double fieldVal)
{
    std::cout << VT100::HIDE_CURSOR;
    // std::cout << VT100::SHOW_CURSOR;
    uint16_t adjustedPrintRow = printRow + TERM::NUM_HEADER_ROWS;
    std::cout << VT100::CURSOR_TO_ROW_COL(adjustedPrintRow, fieldStartCol + 4);
    // std::cout << VT100::CLEAR_TO_END_OF_ROW;
    // cursorTCol(fieldStartCol+4);
    std::cout << std::setprecision(6) << fieldVal;
    std::cout << VT100::CLEAR_TO_END_OF_ROW;
    // std::cout << VT100::SHOW_CURSOR;

    std::cout << std::flush;
}

template <>
inline void TerminalInterface::updatePersistentField(uint8_t printRow, const std::string &fieldValStr)
{
    std::cout << VT100::HIDE_CURSOR;
    // std::cout << VT100::SHOW_CURSOR;
    uint16_t adjustedPrintRow = printRow + TERM::NUM_HEADER_ROWS;
    std::cout << VT100::CURSOR_TO_ROW_COL(adjustedPrintRow, fieldStartCol + 4);
    // std::cout << VT100::CLEAR_TO_END_OF_ROW;
    // cursorTCol(fieldStartCol+4);
    std::cout << fieldValStr;
    std::cout << VT100::CLEAR_TO_END_OF_ROW;
    // std::cout << VT100::SHOW_CURSOR;

    std::cout << std::flush;
}

template <typename T>
inline void TerminalInterface::updatePersistentField(uint8_t printRow, const T val)
{
    std::cout << VT100::HIDE_CURSOR;
    // std::cout << VT100::SHOW_CURSOR;
    uint16_t adjustedPrintRow = printRow + TERM::NUM_HEADER_ROWS;
    std::cout << VT100::CURSOR_TO_ROW_COL(adjustedPrintRow, fieldStartCol + 4);
    // std::cout << VT100::CLEAR_TO_END_OF_ROW;
    // cursorTCol(fieldStartCol+4);
    std::cout << val;
    std::cout << VT100::CLEAR_TO_END_OF_ROW;
    // std::cout << VT100::SHOW_CURSOR;

    std::cout << std::flush;
}