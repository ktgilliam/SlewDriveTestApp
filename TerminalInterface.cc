
#include "TerminalInterface.h"

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdio.h>

#include <cinttypes>
// #include <mathFuncs.h>
#include <cstring>
#include <stdio.h>
#include <iostream>
// #include <patch.h>

//   collector(std::unique_ptr<std::ostringstream>(new std::ostringstream))
TerminalInterface::TerminalInterface(const std::string &_label)
    : ifLabel(_label)
{
    std::cout << VT100::CLEAR_CONSOLE;
    debugRowOffset = 0;
    debugMessageCount = 0;
    promptRow = TERM::NUM_HEADER_ROWS + 1;
    printHeader();
    VT100::SET_WINDOW_SIZE(40, TERMINAL_WIDTH);
    // resetPrompt();
};

void TerminalInterface::printHeader()
{

    std::string HEADER_BORDER_STRING = std::string(TERMINAL_WIDTH, '#');
    std::string HEADER_LABEL_STRING = ifLabel;
    std::string HEADER_LABEL_ROW_SIDE = std::string((TERMINAL_WIDTH - HEADER_LABEL_STRING.size()) / 2 - 1, '#');
    std::string HEADER_LABEL_ROW = HEADER_LABEL_ROW_SIDE + " " + HEADER_LABEL_STRING + " " + HEADER_LABEL_ROW_SIDE;

    std::cout << VT100::HOME << VT100::WHITE;
    std::cout << HEADER_BORDER_STRING;
    std::cout << VT100::CURSOR_TO_ROW(TERM::MIDDLE_HEADER);
    std::cout << HEADER_LABEL_ROW;
    std::cout << VT100::CURSOR_TO_ROW(TERM::LOWER_HEADER);
    std::cout << HEADER_BORDER_STRING;
    std::cout << std::flush;
}

void TerminalInterface::resetPrompt()
{

    messageRow = promptRow + 2;
    std::cout << VT100::CURSOR_TO_ROW_COL(messageRow, 0);
    std::string DEBUG_BORDER_STR = std::string(TERMINAL_WIDTH, '-');
    std::cout << DEBUG_BORDER_STR;

    std::cout << VT100::SHOW_CURSOR;
    std::memset(rxBuff, '\0', CLI_BUFF_LENGTH);
    rxPtr = rxBuff;
    std::cout << VT100::CURSOR_TO_ROW(promptRow);
    std::cout << ">> ";
    std::cout << VT100::CLEAR_TO_END_OF_ROW;
    currentInputCol = 4;
    std::cout << VT100::CURSOR_TO_COL(currentInputCol);

    std::cout << std::flush;
    // BLINKING();
}

void TerminalInterface::serviceCLI()
{
#if PRINT_SERVICE_COUNTER
    static uint64_t serviceCounter = 0;
    std::cout << VT100::CURSOR_TO_ROW_COL(SERVICE_COUNTER_ROW, 0);
    printf("[%o]", serviceCounter++);
#endif
    // static int64_t cnt =0;
    std::cout << VT100::CURSOR_TO_ROW_COL(promptRow, currentInputCol);
    // std::string tmp;
    // char c;
    // std::cin >> c;

    // std::string line;
    // std::getline(std::cin, line);

    // addDebugMessage(tmp, TERM::INFO);

    // if (available() > 0)
    // {
    //     // read the incoming byte:
    //     char c = read();

    //     if (c == '\r' || c == '\n')
    //     {
    //         handleCliCommand();
    //     }
    //     else
    //     {
    //         // say what you got:
    //         printf("%c", c);
    //         // Put it in the buffer
    //         if (rxPtr < (rxBuff + CLI_BUFF_LENGTH - 1))
    //         {
    //             *rxPtr++ = c;
    //             currentInputCol++;
    //         }
    //     }
    // cnt = 0;
    // }
    // else
    // {
    // printf("%d", cnt++);
    // }
}


void inputCallback (std::ios::event ev, std::ios_base& stream, int index)
{
  switch (ev)
  {
    case stream.copyfmt_event:
      std::cout << "copyfmt_event\n"; break;
    case stream.imbue_event:
      std::cout << "imbue_event\n"; break;
    case stream.erase_event:
      std::cout << "erase_event\n"; break;
  }
}


void TerminalInterface::handleCliCommand()
{
    std::cout << VT100::CURSOR_TO_ROW(promptRow + 1);
    std::cout << VT100::CLEAR_TO_END_OF_ROW;
    printf("%s: Command Not Found.\r\n", rxBuff);

    resetPrompt();
}

void TerminalInterface::addPersistentField(const std::string &label, uint8_t printRow)
{
    static uint16_t highestFieldRowNum = 0;
    uint16_t adjustedPrintRow = printRow + TERM::NUM_HEADER_ROWS;
    if (adjustedPrintRow > highestFieldRowNum)
    {
        highestFieldRowNum = adjustedPrintRow;
        promptRow = highestFieldRowNum + 3;
        firstDebugRow = promptRow + 3;
    }
    if (fieldStartCol < (label.size() + 1))
    {
        fieldStartCol = label.size() + 1;
    }
    PersistentTerminalField *field = new PersistentTerminalField();
    field->printRow = adjustedPrintRow;
    field->label = label;
    persistentFields.push_back(field);
    // resetPrompt();
}

void TerminalInterface::printPersistentFieldLabels()
{
    for (auto field : persistentFields)
    {
        std::cout << VT100::CURSOR_TO_ROW(field->printRow) << VT100::CLEAR_TO_END_OF_ROW;
        std::cout << field->label << VT100::CURSOR_TO_COL(22);
    }

    std::cout << std::flush;
    resetPrompt();
}

void TerminalInterface::addDebugMessage(const std::string &msg, uint8_t level)
{
    debugMessageCount++;
    const char *colorStr;
    switch (level)
    {
    case TERM::INFO:
        colorStr = VT100::WHITE;
        break;
    case TERM::DEBUG:
        colorStr = VT100::GREEN;
        break;
    case TERM::WARNING:
        colorStr = VT100::YELLOW;
        break;
    case TERM::ERROR:
        colorStr = VT100::RED;
        break;
    }
    std::stringstream ss;
    ss << std::setiosflags(std::ios::left) << std::setw(6);
    ss << VT100::WHITE << debugMessageCount << ": " << colorStr << msg;
    std::string msgPrintSr = ss.str();

    if (debugMessages.size() < TERM::MAX_DEBUG_ROWS)
    {
        std::cout << VT100::CURSOR_TO_ROW_COL((firstDebugRow + debugRowOffset++), 0);
        std::cout << VT100::CLEAR_TO_END_OF_ROW;
        debugMessages.push_back(msgPrintSr);
        std::cout << msgPrintSr << std::endl;
    }
    else
    {
        debugMessages.pop_front();
        debugMessages.push_back(msgPrintSr);
        for (uint16_t ii = 0; ii < TERM::MAX_DEBUG_ROWS; ii++)
        {
            std::cout << VT100::CURSOR_TO_ROW_COL((firstDebugRow + ii), 0);
            std::cout << VT100::CLEAR_TO_END_OF_ROW;
            std::cout << debugMessages.at(ii);
            // printf(" [fdr:%d][dro:%d][ii:%d]", firstDebugRow, debugRowOffset, ii);
        }
    }

    std::cout << std::flush;
}

int fs_sexa(char *out, double a, int w, int fracbase)
{
    char *out0 = out;
    unsigned long n;
    int d;
    int f;
    int m;
    int s;
    int isneg;

    /* save whether it's negative but do all the rest with a positive */
    isneg = (a < 0);
    if (isneg)
        a = -a;

    /* convert to an integral number of whole portions */
    n = (unsigned long)(a * fracbase + 0.5);
    d = n / fracbase;
    f = n % fracbase;

    /* form the whole part; "negative 0" is a special case */
    if (isneg && d == 0)
        out += snprintf(out, TERM::MAX_CLOCKBUFF_LEN, "%*s-0", w - 2, "");
    else
        out += snprintf(out, TERM::MAX_CLOCKBUFF_LEN, "%*d", w, isneg ? -d : d);

    /* do the rest */
    switch (fracbase)
    {
    case 60: /* dd:mm */
        m = f / (fracbase / 60);
        out += snprintf(out, TERM::MAX_CLOCKBUFF_LEN, ":%02d", m);
        break;
    case 600: /* dd:mm.m */
        out += snprintf(out, TERM::MAX_CLOCKBUFF_LEN, ":%02d.%1d", f / 10, f % 10);
        break;
    case 3600: /* dd:mm:ss */
        m = f / (fracbase / 60);
        s = f % (fracbase / 60);
        out += snprintf(out, TERM::MAX_CLOCKBUFF_LEN, ":%02d:%02d", m, s);
        break;
    case 36000: /* dd:mm:ss.s*/
        m = f / (fracbase / 60);
        s = f % (fracbase / 60);
        out += snprintf(out, TERM::MAX_CLOCKBUFF_LEN, ":%02d:%02d.%1d", m, s / 10, s % 10);
        break;
    case 360000: /* dd:mm:ss.ss */
        m = f / (fracbase / 60);
        s = f % (fracbase / 60);
        out += snprintf(out, TERM::MAX_CLOCKBUFF_LEN, ":%02d:%02d.%02d", m, s / 100, s % 100);
        break;
    default:
        printf("fs_sexa: unknown fracbase: %d\n", fracbase);
        return -1;
    }

    return (out - out0);
}
