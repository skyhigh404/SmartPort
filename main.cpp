#include "gameManager.h"
#include "scheduler.h"

int main()
{
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
