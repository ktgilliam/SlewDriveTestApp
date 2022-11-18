#pragma once

#include <unistd.h>
#include <string>
#include <vector>

#define READ_ONLY_FD 0
#define WRITE_ONLY_FD 1

struct PipeWrapper
{
    int fileDescriptors[2];
    void close();
    void close(int rw);
};

class BashCommand
{
public:
    int ExitStatus = 0;
    std::string Command;
    std::string StdIn;
    std::string StdOut;
    std::string StdErr;
    std::vector<std::string> delimittedData;

    BashCommand() {}

    void execBashCommandWithPipes();
    static std::vector<std::string> splitByDelimeter(std::string s, std::string delimeter);

    int delimittedCopyPipeContents(PipeWrapper roPipe, std::vector<std::string> _delimittedData);

private:
    PipeWrapper stdInPipe;
    PipeWrapper stdOutPipe;
    PipeWrapper stdErrPipe;

    static bool readLineFromPipe(PipeWrapper roPipe, std::string &dest);
    void createPipes();
    int createFork();
    void copyPipeContents(PipeWrapper roPipe, std::string &dest);

    void cleanUp();
};