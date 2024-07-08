#include "Subscribe_Share.h"
#include "CEM_subs.hpp"

CEMSubscription subscriber_cem;

void subscribe_share()
{
    subscriber_cem.subscribe_CEM();
}


void reset_subscription_containers(void)
{
    subscriber_cem.reset_subscription_containers_CEM();
}
