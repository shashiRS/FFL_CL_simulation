#pragma once
#ifndef __INCLUDE_CEM_SUBS_H
#define __INCLUDE_CEM_SUBS_H

#include <ecal/ecal.h>

class CEMSubscription : public std::enable_shared_from_this<CEMSubscription> {
public:
	void subscribe_CEM();
	void reset_subscription_containers_CEM();
};

#endif /* __INCLUDE_CEM_SUBS_H */
