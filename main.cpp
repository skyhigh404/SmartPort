#include "gameManager.h"
#include "scheduler.h"
#include "log.h"

int main()
{
#ifdef DEBUG
    Log::initLog("log/log.log");
#endif

    GameManager gameManager;
    // gameManager.setScheduler();
    gameManager.initializeGame();
    while (1)
    {
        gameManager.processFrameData();
        gameManager.update();
        gameManager.outputCommands();
    }

    return 0;
}
