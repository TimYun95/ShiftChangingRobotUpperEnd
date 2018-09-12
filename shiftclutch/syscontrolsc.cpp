#include "syscontrolsc.h"

syscontrolsc::syscontrolsc() {
    // 给个初始值
    GearStatus = gearstatus::Auto;
    ClutchStatus = clutchstatus::Released;
    ManualShiftStatus = manualshiftstatus::Gear_N;
    AutoShiftStatus = autoshiftstatus::Gear_N;
    ShiftChangingMode = shiftchangingmode::AutoShift;
    ShiftChangingStatus = shiftchangingstatus::Idle;
    LastShiftChangingStatus = shiftchangingstatus::Idle;
    ClutchReleasingMode = clutchreleasingmode::Normal;

    shiftpositions.clear();
    shiftpositions.reserve(16);
    clutchpositons[0] = 0; clutchpositons[1] = 0;
    angletolerance[0] = 1; angletolerance[1] = 1; angletolerance[2] = 1;
    percenttolerance = 0.1;
    curvemotionspeed[0] = 1; curvemotionspeed[1] = 1; curvemotionspeed[2] = 1;
    slowlyreleasingclutchspeedatdeparture = 1;
    slowlyreleasingclutchspeed = 1;
    accstartangle = 20;
    pedalrecoverypercent[0] = 1; pedalrecoverypercent[1] = 1;
    pedalrecoveryrecord[0] = 0; pedalrecoveryrecord[1] = 0;

    shiftchanginglist.clear();
    shiftchanginglist.reserve(8);
    shiftchanginglength = 1;
    shiftchangingpointer = 0;
    currentshiftindex = 0;
    aimshiftindex = 0;
    currentclutchindex = 0;
    aimclutchindex = 0;
    round = 1;
    isshiftchangingplanned = false;

    gettimeofday(&sectionstarttime, NULL);
    gettimeofday(&sectionstoptime, NULL);

    sectionsusingtime[0] = 0;
    sectionsusingtime[1] = 0;
    sectionsusingtime[2] = 0;
    sectionsusingtime[3] = 0;
    sectionsusingtime[4] = 0;
}

bool syscontrolsc::settingshiftchanginginfos(
        bool isManualGear,
        doublepair teachedshiftpositions,
        double teachedclutchpositions[],
        double givenangletolerance[],
        double givenpercenttolerance,
        double givencurvemotionspeed[],
        double givenslowlyreleasingclutchspeed[],
        double givenaccstartangle,
        double givenpedalrecoverypercent[]) {
    GearStatus = isManualGear ? gearstatus::Manual : gearstatus::Auto;
    ClutchStatus = clutchstatus::Released;
    ManualShiftStatus = manualshiftstatus::Gear_N;
    AutoShiftStatus = autoshiftstatus::Gear_N;
    ShiftChangingMode = shiftchangingmode::OnlyClutch;
    ShiftChangingStatus = shiftchangingstatus::Idle;
    LastShiftChangingStatus = shiftchangingstatus::Idle;
    ClutchReleasingMode = clutchreleasingmode::Normal;

    shiftpositions.clear();
    shiftpositions.reserve(16);
    unsigned int lengthofteachedshiftpositions = teachedshiftpositions.size();
    for (unsigned int i = 0; i < lengthofteachedshiftpositions; ++i) {
        shiftpositions.push_back( std::make_pair(teachedshiftpositions[i].first, teachedshiftpositions[i].second) );
    }
    clutchpositons[0] = teachedclutchpositions[0];
    clutchpositons[1] = teachedclutchpositions[1];
    angletolerance[0] = givenangletolerance[0];
    angletolerance[1] = givenangletolerance[1];
    angletolerance[2] = givenangletolerance[2];
    percenttolerance = givenpercenttolerance;
    curvemotionspeed[0] = givencurvemotionspeed[0];
    curvemotionspeed[1] = givencurvemotionspeed[1];
    curvemotionspeed[2] = givencurvemotionspeed[2];
    slowlyreleasingclutchspeedatdeparture = givenslowlyreleasingclutchspeed[0];
    slowlyreleasingclutchspeed = givenslowlyreleasingclutchspeed[1];
    accstartangle = givenaccstartangle;
    pedalrecoverypercent[0] = givenpedalrecoverypercent[0];
    pedalrecoverypercent[1] = givenpedalrecoverypercent[1];

    shiftchanginglist.clear();
    shiftchanginglist.reserve(8);
    shiftchanginglength = 1;
    shiftchangingpointer = 0;

    if (isManualGear) {
        currentshiftindex = (int)manualshiftstatus::Gear_N;
        aimshiftindex = (int)manualshiftstatus::Gear_1;
    }
    else {
        currentshiftindex = (int)autoshiftstatus::Gear_N;
        aimshiftindex = (int)autoshiftstatus::Gear_D;
    }
    currentclutchindex = (int)clutchstatus::Released;
    aimclutchindex = (int)clutchstatus::Pressed;
    round = 1;
    isshiftchangingplanned = false;

    gettimeofday(&sectionstarttime, NULL);
    gettimeofday(&sectionstoptime, NULL);

    sectionsusingtime[0] = 0;
    sectionsusingtime[1] = 0;
    sectionsusingtime[2] = 0;
    sectionsusingtime[3] = 0;
    sectionsusingtime[4] = 0;

    return true;
}

bool syscontrolsc::resetshiftchangingprocess(bool isManualGear)
{
    GearStatus = isManualGear ? gearstatus::Manual : gearstatus::Auto;
    ClutchStatus = clutchstatus::Released;
    ManualShiftStatus = manualshiftstatus::Gear_N;
    AutoShiftStatus = autoshiftstatus::Gear_N;
    ShiftChangingMode = shiftchangingmode::OnlyClutch;
    ShiftChangingStatus = shiftchangingstatus::Idle;
    LastShiftChangingStatus = shiftchangingstatus::Idle;
    ClutchReleasingMode = clutchreleasingmode::Normal;

    shiftchanginglist.clear();
    shiftchanginglist.reserve(8);
    shiftchanginglength = 1;
    shiftchangingpointer = 0;

    if (isManualGear) {
        currentshiftindex = (int)manualshiftstatus::Gear_N;
        aimshiftindex = (int)manualshiftstatus::Gear_1;
    }
    else {
        currentshiftindex = (int)autoshiftstatus::Gear_N;
        aimshiftindex = (int)autoshiftstatus::Gear_D;
    }
    currentclutchindex = (int)clutchstatus::Released;
    aimclutchindex = (int)clutchstatus::Pressed;
    round = 1;
    isshiftchangingplanned = false;

    gettimeofday(&sectionstarttime, NULL);
    gettimeofday(&sectionstoptime, NULL);

    sectionsusingtime[0] = 0;
    sectionsusingtime[1] = 0;
    sectionsusingtime[2] = 0;
    sectionsusingtime[3] = 0;
    sectionsusingtime[4] = 0;

    return true;
}

syscontrolsc::doublevector syscontrolsc::getshiftchangingangles(
        double actualangles[],
        int howtochangeshift,
        double pedallowlimits[],
        int aimshift,
        int aimclutch,
        int howtoreleaseclutch,
        double *pedalcommand,
        bool ifpaused) {
    if (ifpaused)
    {
        double stopplaceholder[6] = {-100, -100, -100, -100, -100, -100};
        return makefeedbackvector(stopplaceholder);
    }

    if (!isshiftchangingplanned) {
        isshiftchangingplanned = true;
        bool issuccessplan = planshiftchangingsteps(
                    aimshift,
                    shiftchangingmode(howtochangeshift),
                    clutchreleasingmode(howtoreleaseclutch),
                    clutchstatus(aimclutch));
        if (!issuccessplan) {
            isshiftchangingplanned = false;
            double stopplaceholder[6] = {-100, -100, -100, -100, -100, -100};
            return makefeedbackvector(stopplaceholder);
        }
        pedalrecoveryrecord[0] = actualangles[0];
        pedalrecoveryrecord[1] = actualangles[1];
    }

    if (shiftchangingpointer >= shiftchanginglength) {
        round = 1;
        LastShiftChangingStatus = ShiftChangingStatus;
        double stopplaceholder[6] = {-100, -100, -100, -100, -100, -100};
        return makefeedbackvector(stopplaceholder);
    }

    ShiftChangingStatus = shiftchanginglist[shiftchangingpointer].first;
    if (ShiftChangingStatus != LastShiftChangingStatus) {
        round = 1;
        LastShiftChangingStatus = ShiftChangingStatus;
        gettimeofday(&sectionstarttime, NULL);
    }
    if (ShiftChangingStatus == shiftchangingstatus::ShiftChanging)
    {
        if (aimshiftindex == currentshiftindex)
        {
            round = 1;
            gettimeofday(&sectionstarttime, NULL);
        }
    }
    switch (ShiftChangingStatus) {
    case shiftchangingstatus::ClutchPressing: {
        aimclutchindex = (int)clutchstatus::Pressed;
        double clutchcmd = getinterpclutch();

        switch (shiftchangingmode(howtochangeshift)) {
        case shiftchangingmode::OnlyClutch: {
            bool ifachievedgoal = ifreachedaim(
                        true,
                        (int)clutchstatus::Pressed,
                        actualangles);

            if (ifachievedgoal) {
                gettimeofday(&sectionstoptime, NULL);
                sectionsusingtime[shiftchangingpointer] = caltimeinterval();

                shiftchangingpointer++;

                ClutchStatus = clutchstatus::Pressed;
                currentclutchindex = (int)ClutchStatus;

                ShiftChangingStatus = shiftchangingstatus::Idle;
                isshiftchangingplanned = false;

                double stopplaceholder[6] = {-100, -100, -100, -100, -100, -100};
                return makefeedbackvector(stopplaceholder);
            }
            else {
                double anglescmd[6] = {-100, -100,
                                      clutchcmd,
                                      -100, -100, -100};
                return makefeedbackvector(anglescmd);
            }
            break;
        }
        case shiftchangingmode::ThreeAxis: {
            bool ifachievedgoal = ifreachedaim(
                        true,
                        (int)clutchstatus::Pressed,
                        actualangles);

            if (ifachievedgoal) {
                gettimeofday(&sectionstoptime, NULL);
                sectionsusingtime[shiftchangingpointer] = caltimeinterval();

                shiftchangingpointer++;

                ClutchStatus = clutchstatus::Pressed;
                currentclutchindex = (int)ClutchStatus;

                return getshiftchangingangles(
                            actualangles,
                            howtochangeshift,
                            pedallowlimits,
                            aimshift,
                            aimclutch,
                            howtoreleaseclutch,
                            pedalcommand,
                            ifpaused);
            }
            else {
                double anglescmd[6] = {-100, -100,
                                      clutchcmd,
                                      -100, -100, -100};
                return makefeedbackvector(anglescmd);
            }
            break;
        }
        case shiftchangingmode::FiveAxis:
        case shiftchangingmode::ManualShift: {
            bool ifachievedgoal = ifreachedaim(
                        true,
                        (int)clutchstatus::Pressed,
                        actualangles,
                        true,
                        pedallowlimits);

            if (ifachievedgoal) {
                gettimeofday(&sectionstoptime, NULL);
                sectionsusingtime[shiftchangingpointer] = caltimeinterval();

                shiftchangingpointer++;

                ClutchStatus = clutchstatus::Pressed;
                currentclutchindex = (int)ClutchStatus;

                return getshiftchangingangles(
                            actualangles,
                            howtochangeshift,
                            pedallowlimits,
                            aimshift,
                            aimclutch,
                            howtoreleaseclutch,
                            pedalcommand,
                            ifpaused);
            }
            else {
                double anglescmd[6] = {
                                      pedallowlimits[0],
                                      pedallowlimits[1],
                                      clutchcmd,
                                      -100, -100, -100};
                return makefeedbackvector(anglescmd);
            }
            break;
        }
        default:
            break;
        }
        break;
    }
    case shiftchangingstatus::ShiftChanging: {
        aimshiftindex = shiftchanginglist[shiftchangingpointer].second;
        doublecouple shiftcmd = getinterpshift();

        bool islastchange = shiftchangingpointer+1 == shiftchanginglength ? true : false;
        bool islastshift = false;
        if (islastchange) {
            islastshift = true;
        }
        else {
            if (shiftchanginglist[shiftchangingpointer + 1].first != shiftchangingstatus::ShiftChanging) {
                islastshift = true;
            }
        }

        bool ifachievedgoal = false;
        if (islastshift) {
            ifachievedgoal = ifreachedaim(
                        false,
                        aimshiftindex,
                        actualangles,
                        false,
                        NULL,
                        true);
        }
        else {
            ifachievedgoal = ifreachedaim(
                        false,
                        aimshiftindex,
                        actualangles);
        }

        if (ifachievedgoal) {
            gettimeofday(&sectionstoptime, NULL);
            sectionsusingtime[shiftchangingpointer] = caltimeinterval();

            if (GearStatus == gearstatus::Auto) {
                AutoShiftStatus = autoshiftstatus(shiftchanginglist[shiftchangingpointer].second);
                currentshiftindex = (int)AutoShiftStatus;
            }
            else {
                ManualShiftStatus = manualshiftstatus(shiftchanginglist[shiftchangingpointer].second);
                currentshiftindex = (int)ManualShiftStatus;
            }

            shiftchangingpointer++;

            if (islastchange) {
                ShiftChangingStatus = shiftchangingstatus::Idle;
                isshiftchangingplanned = false;

                double stopplaceholder[6] = {-100, -100, -100, -100, -100, -100};
                return makefeedbackvector(stopplaceholder);
            }
            else {
                return getshiftchangingangles(
                            actualangles,
                            howtochangeshift,
                            pedallowlimits,
                            aimshift,
                            aimclutch,
                            howtoreleaseclutch,
                            pedalcommand,
                            ifpaused);
            }
        }
        else {
            double anglescmd[6] = {-100, -100, -100,
                                  shiftcmd.first,
                                  shiftcmd.second,
                                  -100};
            return makefeedbackvector(anglescmd);
        }
        break;
    }
    case shiftchangingstatus::ClutchReleasing: {
        aimclutchindex = (int)clutchstatus::Released;
        double clutchcmd = getinterpclutch();

        bool ifachievedgoal = ifreachedaim(
                    true,
                    (int)clutchstatus::Released,
                    actualangles);

        if (ifachievedgoal) {
            gettimeofday(&sectionstoptime, NULL);
            sectionsusingtime[shiftchangingpointer] = caltimeinterval();

            shiftchangingpointer++;

            ClutchStatus = clutchstatus::Released;
            currentclutchindex = (int)ClutchStatus;

            ShiftChangingStatus = shiftchangingstatus::Idle;
            isshiftchangingplanned = false;

            double stopplaceholder[6] = {-100, -100, -100, -100, -100, -100};
            return makefeedbackvector(stopplaceholder);
        }
        else {
            double anglescmd[6] = {-100, -100,
                                  clutchcmd,
                                  -100, -100, -100};
            return makefeedbackvector(anglescmd);
        }
        break;
    }
    case shiftchangingstatus::ClutchReleasingSlowly: {
        aimclutchindex = (int)clutchstatus::Released;
        ClutchStatus = clutchstatus::SlowlyReleasing;
        double clutchcmd;

        if (shiftchangingmode(howtochangeshift) == shiftchangingmode::OnlyClutch)
        {
            if (shiftchanginglist[shiftchangingpointer].second != 0)
            {
                clutchcmd = getinterpclutch(1);
            }
            else
            {
                clutchcmd = getinterpclutch(2);
            }
        }
        else
        {
            if (ManualShiftStatus == manualshiftstatus::Gear_1) {
                clutchcmd = getinterpclutch(1);
            }
            else {
                clutchcmd = getinterpclutch(2);
            }
        }

        bool ifachievedgoal = ifreachedaim(
                    true,
                    (int)clutchstatus::Released,
                    actualangles);

        if (ifachievedgoal)
        {
            gettimeofday(&sectionstoptime, NULL);
            sectionsusingtime[shiftchangingpointer] = caltimeinterval();

            shiftchangingpointer++;

            ClutchStatus = clutchstatus::Released;
            currentclutchindex = (int)ClutchStatus;

            ShiftChangingStatus = shiftchangingstatus::Idle;
            isshiftchangingplanned = false;

            double stopplaceholder[6] = {-100, -100, -100, -100, -100, -100};
            return makefeedbackvector(stopplaceholder);
        }
        else
        {
            if ( ManualShiftStatus == manualshiftstatus::Gear_1 && (howtochangeshift == (int)shiftchangingmode::FiveAxis || howtochangeshift == (int)shiftchangingmode::ManualShift) )
            {
                double clutchrunningpercent = 1.0 - (clutchcmd - clutchpositons[(int)clutchstatus::Released]) / (clutchpositons[(int)clutchstatus::Pressed] - clutchpositons[(int)clutchstatus::Released]);
                double accdelta = accstartangle - pedallowlimits[1];
                double acccmd = clutchrunningpercent * pedalrecoverypercent[1] * accdelta + pedallowlimits[1];

                double anglescmd[6] = {-100,
                                      acccmd,
                                      clutchcmd,
                                      -100, -100, -100};
                return makefeedbackvector(anglescmd);
            }
            else
            {
                double anglescmd[6] = {-100, -100,
                                      clutchcmd,
                                      -100, -100, -100};
                return makefeedbackvector(anglescmd);
            }
        }
        break;
    }
    case shiftchangingstatus::ClutchReleasingSlowlyWithPedalControl: {
        aimclutchindex = (int)clutchstatus::Released;
        ClutchStatus = clutchstatus::SlowlyReleasing;
        double clutchcmd;
        if (ManualShiftStatus == manualshiftstatus::Gear_1)
        {
            clutchcmd = getinterpclutch(1);
        }
        else
        {
            clutchcmd = getinterpclutch(2);
        }

        bool ifachievedgoal = ifreachedaim(
                    true,
                    (int)clutchstatus::Released,
                    actualangles);

        if (ifachievedgoal)
        {
            gettimeofday(&sectionstoptime, NULL);
            sectionsusingtime[shiftchangingpointer] = caltimeinterval();

            shiftchangingpointer++;

            ClutchStatus = clutchstatus::Released;
            currentclutchindex = (int)ClutchStatus;

            ShiftChangingStatus = shiftchangingstatus::Idle;
            isshiftchangingplanned = false;

            if (!pedalcommand)
            {
                double stopplaceholder[6] = {-100, -100, -100, -100, -100, -100};
                return makefeedbackvector(stopplaceholder);
            }
            else
            {
                double onlypedalplaceholder[6] = {
                    pedalcommand[0], pedalcommand[1],
                    -100, -100, -100, -100};
                return makefeedbackvector(onlypedalplaceholder);
            }
        }
        else {
            if ( ManualShiftStatus == manualshiftstatus::Gear_1 )
            {
                double clutchrunningpercent = 1.0 - (clutchcmd - clutchpositons[(int)clutchstatus::Released]) / (clutchpositons[(int)clutchstatus::Pressed] - clutchpositons[(int)clutchstatus::Released]);
                double accdelta = accstartangle - pedallowlimits[1];
                double acccmd = clutchrunningpercent * pedalrecoverypercent[1] * accdelta + pedallowlimits[1];

                double anglescmd[6] = {-100,
                                      acccmd,
                                      clutchcmd,
                                      -100, -100, -100};
                return makefeedbackvector(anglescmd);
            }
            else
            {
            double anglescmd[6] = {
                                  pedalcommand[0],
                                  pedalcommand[1],
                                  clutchcmd,
                                  -100, -100, -100};
            return makefeedbackvector(anglescmd);
            }
        }
        break;
    }
    case shiftchangingstatus::ClutchReleasingSlowlyWithPedalRecovery: {
        aimclutchindex = (int)clutchstatus::Released;
        ClutchStatus = clutchstatus::SlowlyReleasing;
        double clutchcmd;
        if (ManualShiftStatus == manualshiftstatus::Gear_1)
        {
            clutchcmd = getinterpclutch(1);
        }
        else
        {
            clutchcmd = getinterpclutch(2);
        }

        bool ifachievedgoal = ifreachedaim(
                    true,
                    (int)clutchstatus::Released,
                    actualangles);

        if (ifachievedgoal)
        {
            gettimeofday(&sectionstoptime, NULL);
            sectionsusingtime[shiftchangingpointer] = caltimeinterval();

            shiftchangingpointer++;

            ClutchStatus = clutchstatus::Released;
            currentclutchindex = (int)ClutchStatus;

            ShiftChangingStatus = shiftchangingstatus::Idle;
            isshiftchangingplanned = false;

            if (!pedalcommand)
            {
                double stopplaceholder[6] = {-100, -100, -100, -100, -100, -100};
                return makefeedbackvector(stopplaceholder);
            }
            else
            {
                double onlypedalplaceholder[6] = {
                    pedalcommand[0], pedalcommand[1],
                    -100, -100, -100, -100};
                return makefeedbackvector(onlypedalplaceholder);
            }
        }
        else
        {
            double clutchrunningpercent = 1.0 - (clutchcmd - clutchpositons[(int)clutchstatus::Released]) / (clutchpositons[(int)clutchstatus::Pressed] - clutchpositons[(int)clutchstatus::Released]);
            double brkdelta = pedalrecoveryrecord[0] - pedallowlimits[0];
            double accdelta = pedalrecoveryrecord[1] - pedallowlimits[1];
            double brkcmd = -100;
            double acccmd = -100;
            if (brkdelta >= 3 && accdelta < 3) {
                brkcmd = clutchrunningpercent * pedalrecoverypercent[0] * brkdelta + pedallowlimits[0];
            }
            else if (brkdelta < 3 && accdelta >= 3) {
                acccmd = clutchrunningpercent * pedalrecoverypercent[1] * accdelta + pedallowlimits[1];
            }
            else if (brkdelta >= 3 && accdelta >= 3) {
                acccmd = clutchrunningpercent * pedalrecoverypercent[1] * accdelta + pedallowlimits[1];
            }

            if ( ManualShiftStatus == manualshiftstatus::Gear_1 )
            {
                double anglescmd[6] = {-100,
                                      acccmd,
                                      clutchcmd,
                                      -100, -100, -100};
                return makefeedbackvector(anglescmd);
            }
            else
            {
            double anglescmd[6] = {
                                  brkcmd,
                                  acccmd,
                                  clutchcmd,
                                  -100, -100, -100};
            return makefeedbackvector(anglescmd);
            }
        }
        break;
    }
    default:
        break;
    }

    double errornullplaceholder[6] = {0, 0, 0, 0, 0, 0};
    return makefeedbackvector(errornullplaceholder, true);
}

double syscontrolsc::sgn(double x) {
    if (x > 0.0) return 1;
    else if (x < 0.0) return -1;
    else return 0;
}

syscontrolsc::manualshiftstatus syscontrolsc::getnodeformanualshift(const manualshiftstatus s) {
    switch (s) {
    case manualshiftstatus::Gear_NLeft:
    case manualshiftstatus::Gear_1:
    case manualshiftstatus::Gear_2:
        return manualshiftstatus::Gear_NLeft;
        break;
    case manualshiftstatus::Gear_N:
    case manualshiftstatus::Gear_3:
    case manualshiftstatus::Gear_4:
        return manualshiftstatus::Gear_N;
        break;
    case manualshiftstatus::Gear_NRight:
    case manualshiftstatus::Gear_5:
    case manualshiftstatus::Gear_6:
        return manualshiftstatus::Gear_NRight;
        break;
    case manualshiftstatus::Gear_NBack:
    case manualshiftstatus::Gear_R:
        return manualshiftstatus::Gear_NBack;
        break;
    default:
        PRINTF(LOG_WARNING, "%s: no matched nodeshift.\n", __func__);
        return manualshiftstatus(0);
        break;
    }
}

bool syscontrolsc::planshiftchangingsteps(
        int aimshift,
        shiftchangingmode howtochangeshift,
        clutchreleasingmode howtoreleaseclutch,
        clutchstatus aimclutch) {
    if (howtochangeshift != shiftchangingmode::OnlyClutch)
    {
        if ( (GearStatus == gearstatus::Manual && aimshift == (int)ManualShiftStatus) ||
             (GearStatus == gearstatus::Auto && aimshift == (int)AutoShiftStatus) ) {
            PRINTF(LOG_WARNING, "%s: already at current shift.\n", __func__);
            return false;
        }
    }
    else
    {
        if ( (aimclutch == ClutchStatus) ||
             (ClutchStatus == clutchstatus::Released && aimclutch >= clutchstatus::SlowlyReleasing) ) {
            PRINTF(LOG_WARNING, "%s: already at current clutch.\n", __func__);
            return false;
        }
    }

    if (GearStatus == gearstatus::Manual && (aimshift > (int)manualshiftstatus::Gear_5 && aimshift < (int)manualshiftstatus::Gear_6) ) {
        PRINTF(LOG_WARNING, "%s: shiftaim is abnormal.\n", __func__);
        return false;
    }

    shiftchanginglist.clear();
    shiftchanginglist.reserve(8);

    switch (howtochangeshift) {
    case shiftchangingmode::OnlyClutch: {
        switch (aimclutch) {
        case clutchstatus::Pressed:
            shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ClutchPressing, 0) );
            break;
        case clutchstatus::Released:
            shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ClutchReleasing, 0) );
            break;
        case clutchstatus::SlowlyReleasing:
            shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ClutchReleasingSlowly, 0) );
            break;
        case clutchstatus::SlowlyReleasingAtDeparture:
            shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ClutchReleasingSlowly, 1) );
            break;
        default:
            break;
        }
        break;
    }
    case shiftchangingmode::OnlyShift: {
        manualshiftstatusvector shifttransfer = planmanualshiftchangingpartforshifttransfer(manualshiftstatus(aimshift));
        unsigned int transferlength = shifttransfer.size();
        for (unsigned int i = 0; i < transferlength; ++i) {
            shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ShiftChanging, (int)shifttransfer[i]) );
        }
        break;
    }
    case shiftchangingmode::ThreeAxis:
    case shiftchangingmode::FiveAxis: {
        shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ClutchPressing, 0) );
        manualshiftstatusvector shifttransfer = planmanualshiftchangingpartforshifttransfer(manualshiftstatus(aimshift));
        unsigned int transferlength = shifttransfer.size();
        for (unsigned int i = 0; i < transferlength; ++i) {
            shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ShiftChanging, (int)shifttransfer[i]) );
        }
        shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ClutchReleasingSlowly, 0) );
        break;
    }
    case shiftchangingmode::ManualShift: {
        shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ClutchPressing, 0) );
        manualshiftstatusvector shifttransfer = planmanualshiftchangingpartforshifttransfer(manualshiftstatus(aimshift));
        unsigned int transferlength = shifttransfer.size();
        for (unsigned int i = 0; i < transferlength; ++i) {
            shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ShiftChanging, (int)shifttransfer[i]) );
        }
        switch (howtoreleaseclutch) {
        case clutchreleasingmode::Normal:
            shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ClutchReleasing, 0) );
            break;
        case clutchreleasingmode::Slowly:
            shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ClutchReleasingSlowly, 0) );
            break;
        case clutchreleasingmode::SlowlyWithControl:
            shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ClutchReleasingSlowlyWithPedalControl, 0) );
            break;
        case clutchreleasingmode::SlowlyWithRecovery:
            shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ClutchReleasingSlowlyWithPedalRecovery, 0) );
            break;
        default:
            break;
        }
        break;
    }
    case shiftchangingmode::AutoShift: {
        shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ShiftChanging, aimshift) );
        break;
    }
    default:
        PRINTF(LOG_WARNING, "%s: no matched shift changing method.\n", __func__);
        return false;
        break;
    }

    shiftchanginglength = shiftchanginglist.size();
    shiftchangingpointer = 0;

    ShiftChangingStatus = shiftchangingstatus::Idle;
    LastShiftChangingStatus = shiftchangingstatus::Idle;

    sectionsusingtime[0] = 0;
    sectionsusingtime[1] = 0;
    sectionsusingtime[2] = 0;
    sectionsusingtime[3] = 0;
    sectionsusingtime[4] = 0;

    return true;
}

syscontrolsc::manualshiftstatusvector syscontrolsc::planmanualshiftchangingpartforshifttransfer(manualshiftstatus aimshift) {
    manualshiftstatusvector shifttransfer;
    shifttransfer.clear();
    shifttransfer.reserve(4);

    const manualshiftstatus currentpos = ManualShiftStatus;
    const manualshiftstatus finalaim = aimshift;
    const manualshiftstatus nodeforcurrentpos = getnodeformanualshift(currentpos);
    const manualshiftstatus nodeforfinalaim = getnodeformanualshift(finalaim);

    if (nodeforcurrentpos == nodeforfinalaim)
    {
        shifttransfer.push_back(finalaim);
    }
    else
    {
        if (nodeforcurrentpos == currentpos)
        {
            shifttransfer.push_back(nodeforfinalaim);
            shifttransfer.push_back(finalaim);
        }
        else if (nodeforfinalaim == finalaim)
        {
            shifttransfer.push_back(nodeforcurrentpos);
            shifttransfer.push_back(finalaim);
        }
        else
        {
            shifttransfer.push_back(nodeforcurrentpos);
            shifttransfer.push_back(nodeforfinalaim);
            shifttransfer.push_back(finalaim);
        }
    }

    return shifttransfer;
}

bool syscontrolsc::ifreachedaim(
        bool askclutchornot,
        const int aim,
        double actualangles[],
        bool isincludepedal,
        double *pedalpos,
        bool ifduringprocess) {

    if (isincludepedal) {
        if (pedalpos[0] >= 0 && fabs(actualangles[0] - pedalpos[0]) > 0.5) {
            return false;
        }
        if (pedalpos[1] >= 0 && fabs(actualangles[1] - pedalpos[1]) > 0.5) {
            return false;
        }
    }

    if (askclutchornot) {
        if ( fabs(actualangles[2] - clutchpositons[aim]) < angletolerance[0] )
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else {
        if (ifduringprocess) {
            double err = fmin( percenttolerance * angledistance(shiftpositions[(int)ManualShiftStatus], shiftpositions[aim]), fmin( angletolerance[1], angletolerance[2] ) );
            if ( angledistance(std::make_pair(actualangles[3], actualangles[4]), shiftpositions[aim]) < err ) {
                return true;
            }
            else {
                return false;
            }
        }
        else {
            if ( fabs(actualangles[3] - shiftpositions[aim].first) < angletolerance[1] &&
                 fabs(actualangles[4] - shiftpositions[aim].second) < angletolerance[2])
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }
}

double syscontrolsc::angledistance(doublecouple startangles, doublecouple stopangles) {
   return sqrt( pow( (startangles.first - stopangles.first), 2 ) + pow( (startangles.second - stopangles.second), 2 ) );
}

syscontrolsc::doublecouple syscontrolsc::getinterpshift() {
    const doublecouple startPos = shiftpositions[currentshiftindex];
    const doublecouple stopPos = shiftpositions[aimshiftindex];
    const doublecouple errPos = std::make_pair( fabs(startPos.first - stopPos.first), fabs(startPos.second - stopPos.second) );

    // 分段线性 完全解耦
    const doublecouple speed = std::make_pair( curvemotionspeed[1], curvemotionspeed[2] );
    const doublecouple thresholdround = std::make_pair( ceil(0.9 * errPos.first / speed.first), ceil(0.9 * errPos.second / speed.second) );

    doublecouple pos = std::make_pair( startPos.first + sgn(stopPos.first - startPos.first) * round * speed.first, startPos.second + sgn(stopPos.second - startPos.second) * round * speed.second );

    if ((pos.first - startPos.first)/(stopPos.first - startPos.first) > 1.0) pos.first = stopPos.first;
    else if ((pos.first - startPos.first)/(stopPos.first - startPos.first) > 0.9)
    {
        pos.first = startPos.first + sgn(stopPos.first - startPos.first) * thresholdround.first * speed.first + sgn(stopPos.first - startPos.first) * (round - thresholdround.first) * speed.first / 2;
    }

    if ((pos.second - startPos.second)/(stopPos.second - startPos.second) > 1.0) pos.second = stopPos.second;
    else if ((pos.second - startPos.second)/(stopPos.second - startPos.second) > 0.9)
    {
        pos.second = startPos.second + sgn(stopPos.second - startPos.second) * thresholdround.second * speed.second + sgn(stopPos.second - startPos.second) * (round - thresholdround.second) * speed.second / 2;
    }

    // 为下次预备
    round++;

    // 返回计算结果
    return pos;
}

double syscontrolsc::getinterpclutch(int modeofreleasing) {
    const double startPos = clutchpositons[currentclutchindex];
    const double stopPos = clutchpositons[aimclutchindex];
    const double errPos = fabs(startPos - stopPos);

    // 分段线性
    double speed;
    switch (modeofreleasing) {
    case 1:
        speed = slowlyreleasingclutchspeedatdeparture;
        break;
    case 2:
        speed = slowlyreleasingclutchspeed;
        break;
    default:
        speed = curvemotionspeed[0];
        break;
    }
    double thresholdround = ceil(0.9 * errPos / speed);

    double pos = startPos + sgn(stopPos - startPos) * round * speed;
    if ((pos - startPos)/(stopPos - startPos) > 1.0) pos = stopPos;
    else if ((pos - startPos)/(stopPos - startPos) > 0.9)
    {
        if (modeofreleasing != 1) {
            pos = startPos + sgn(stopPos - startPos) * thresholdround * speed + sgn(stopPos - startPos) * (round - thresholdround) * speed / 2;
        }
    }

    // 为下次预备
    round++;
    // 返回计算结果
    return pos;
}

syscontrolsc::doublevector syscontrolsc::makefeedbackvector(
        double anglescmd[],
        bool wrongexist) {
    doublevector feedbackvector;
    feedbackvector.clear();
    feedbackvector.reserve(16);

    if (wrongexist) {
        feedbackvector.push_back(-1.0);
        feedbackvector.push_back(-1.0);
        feedbackvector.push_back(-1.0);
        feedbackvector.push_back(-1.0);
        feedbackvector.push_back(-1.0);
        feedbackvector.push_back(-1.0);
        feedbackvector.push_back(-1.0);
    }
    else {
        feedbackvector.push_back((int)ShiftChangingStatus);
        feedbackvector.push_back(anglescmd[0]);
        feedbackvector.push_back(anglescmd[1]);
        feedbackvector.push_back(anglescmd[2]);
        feedbackvector.push_back(anglescmd[3]);
        feedbackvector.push_back(anglescmd[4]);
        feedbackvector.push_back(anglescmd[5]);
    }

    if (GearStatus == gearstatus::Auto) {
        feedbackvector.push_back((int)AutoShiftStatus);
        feedbackvector.push_back((int)clutchstatus::Released);
    }
    else {
        feedbackvector.push_back((int)ManualShiftStatus);
        feedbackvector.push_back((int)ClutchStatus);
    }

    double totaltime = 0.0;
    for (int i = 0; i < shiftchangingpointer; ++i) {
        totaltime += sectionsusingtime[i];
    }
    feedbackvector.push_back(totaltime);
    for (int i = 0; i < shiftchangingpointer; ++i) {
        feedbackvector.push_back(sectionsusingtime[i]);
    }

    return feedbackvector;
}

double syscontrolsc::caltimeinterval() {
    return (sectionstoptime.tv_sec - sectionstarttime.tv_sec) * 1000.0 + (sectionstoptime.tv_usec - sectionstarttime.tv_usec) / 1000.0;
}









