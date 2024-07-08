#include "visuclientinterface.h"

VisuClientInterface* VisuClientInterface::smInstance = nullptr;

VisuClientInterface *VisuClientInterface::get()
{
    return smInstance;
}

void VisuClientInterface::setInterface(VisuClientInterface *instance)
{
    smInstance = instance;
}

VisuClientInterface::VisuClientInterface()
{

}

