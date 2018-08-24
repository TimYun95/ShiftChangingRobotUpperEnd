#include "syscontrolsc.h"
#include <math.h>

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
    clutchpositons = {0.0, 0.0};
    angletolerance = {1.0, 1.0, 1.0};
    percenttolerance = 0.1;
    curvemotionspeed = {1, 1, 1};
    slowlyreleasingclutchspeed = 1;
    accstartangle = 20;
    pedalrecoverypercent = {1.0, 1.0};
    pedalrecoveryrecord = {0.0, 0.0};

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

    sectionsusingtime = {0.0, 0.0, 0.0, 0.0, 0.0};
}

bool syscontrolsc::settingshiftchanginginfos(
        bool isManualGear,
        doublepair teachedshiftpositions,
        double teachedclutchpositions[],
        double givenangletolerance[],
        double givenpercenttolerance,
        double givencurvemotionspeed[],
        double givenslowlyreleasingclutchspeed,
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
    slowlyreleasingclutchspeed = givenslowlyreleasingclutchspeed;
    accstartangle = givenaccstartangle;
    pedalrecoverypercent[0] = givenpedalrecoverypercent[0];
    pedalrecoverypercent[1] = givenpedalrecoverypercent[1];

    shiftchanginglist.clear();
    shiftchanginglist.reserve(8);
    shiftchanginglength = 1;
    shiftchangingpointer = 0;

    if (isManualGear) {
        currentshiftindex = manualshiftstatus::Gear_N;
        aimshiftindex = manualshiftstatus::Gear_1;
    }
    else {
        currentshiftindex = autoshiftstatus::Gear_N;
        aimshiftindex = autoshiftstatus::Gear_D;
    }
    currentclutchindex = clutchstatus::Released;
    aimclutchindex = clutchstatus::Pressed;
    round = 1;
    isshiftchangingplanned = false;

    gettimeofday(&sectionstarttime, NULL);
    gettimeofday(&sectionstoptime, NULL);

    sectionsusingtime = {0.0, 0.0, 0.0, 0.0, 0.0};

    return true;
}

syscontrolsc::doublevector syscontrolsc::getshiftchangingangles(
        double actualangles[],
        int howtochangeshift,
        int aimshift,
        int aimclutch,
        int howtoreleaseclutch,
        double pedalcommand[],
        bool isabscommand[],
        bool ifpaused) {
    if (!isshiftchangingplanned) {
        isshiftchangingplanned = true;
        bool issuccessplan = planshiftchangingsteps(
                    aimshift,
                    shiftchangingmode(howtochangeshift),
                    clutchreleasingmode(howtoreleaseclutch),
                    clutchstatus(aimclutch));
        if (!issuccessplan) {
            double nullplaceholder = {0, 0, 0, 0, 0, 0};
            return makefeedbackvector(nullplaceholder, true);
        }
    }

    if (shiftchangingpointer >= shiftchanginglength) {
        round = 1;
        LastShiftChangingStatus = ShiftChangingStatus;
        double stopplaceholder = {-1, -1, -1, -1, -1, -1};
        return makefeedbackvector(stopplaceholder);
    }

    ShiftChangingStatus = shiftchanginglist[shiftchangingpointer].first;
    if (ShiftChangingStatus != LastShiftChangingStatus) {
        round = 1;
        LastShiftChangingStatus = ShiftChangingStatus;
        gettimeofday(&sectionstarttime, NULL);
    }
    switch (ShiftChangingStatus) {
    case shiftchangingstatus::ClutchPressing: {
        aimclutchindex = clutchstatus::Pressed;
        double clutchcmd = getinterpclutch();
        bool ifachievedgoal = ifreachedaim(
                    true,
                    clutchstatus::Pressed,
                    actualangles);

        switch (shiftchangingmode(howtochangeshift)) {
        case shiftchangingmode::OnlyClutch: {
            if (ifachievedgoal) {
                gettimeofday(&sectionstoptime, NULL);
                sectionsusingtime[shiftchangingpointer] = caltimeinterval();

                shiftchangingpointer++;

                ClutchStatus = clutchstatus::Pressed;
                currentclutchindex = ClutchStatus;

                ShiftChangingStatus = shiftchangingstatus::Idle;

                double stopplaceholder = {-1, -1, -1, -1, -1, -1};
                return makefeedbackvector(stopplaceholder);
            }
            else {
                double anglescmd[6] = {-1, -1,
                                      clutchcmd,
                                      -1, -1, -1};
                return makefeedbackvector(anglescmd);
            }
            break;
        }
        case shiftchangingmode::ThreeAxis: {
            if (ifachievedgoal) {
                gettimeofday(&sectionstoptime, NULL);
                sectionsusingtime[shiftchangingpointer] = caltimeinterval();

                shiftchangingpointer++;

                ClutchStatus = clutchstatus::Pressed;
                currentclutchindex = ClutchStatus;

                ShiftChangingStatus = shiftchangingstatus::Idle;
                // 改到这里
                double stopplaceholder = {-1, -1, -1, -1, -1, -1};
                return makefeedbackvector(stopplaceholder);
            }
            else {
                double anglescmd[6] = {-1, -1,
                                      clutchcmd,
                                      -1, -1, -1};
                return makefeedbackvector(anglescmd);
            }
            break;
        }
        default:
            break;
        }





        break;
    }
    default:
        break;
    }











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
    case manualshiftstatus::Gear_NReverse:
    case manualshiftstatus::Gear_R:
        return manualshiftstatus::Gear_NReverse;
        break;
    default:
        PRINTF(LOG_WARNING, "%s: no matched nodeshift.\n", __func__);
        return manualshiftstatus(1);
        break;
    }
}

bool syscontrolsc::planshiftchangingsteps(
        int aimshift,
        shiftchangingmode howtochangeshift,
        clutchreleasingmode howtoreleaseclutch,
        clutchstatus aimclutch) {
    if ( (GearStatus == gearstatus::Manual && aimshift == ManualShiftStatus) ||
         (GearStatus == gearstatus::Auto && aimshift == AutoShiftStatus) ) {
        PRINTF(LOG_WARNING, "%s: already at current shift.\n", __func__);
        return false;
    }

    if (GearStatus == gearstatus::Manual && (aimshift > manualshiftstatus::Gear_5 && aimshift < manualshiftstatus::Gear_6) ) {
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
        default:
            break;
        }
        break;
    }
    case shiftchangingmode::OnlyShift: {
        manualshiftstatusvector shifttransfer = planmanualshiftchangingpartforshifttransfer(manualshiftstatus(aimshift));
        unsigned int transferlength = shifttransfer.size();
        for (unsigned int i = 0; i < transferlength; ++i) {
            shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ShiftChanging, shifttransfer[i]) );
        }
        break;
    }
    case shiftchangingmode::ThreeAxis:
    case shiftchangingmode::FiveAxis: {
        shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ClutchPressing, 0) );
        manualshiftstatusvector shifttransfer = planmanualshiftchangingpartforshifttransfer(manualshiftstatus(aimshift));
        unsigned int transferlength = shifttransfer.size();
        for (unsigned int i = 0; i < transferlength; ++i) {
            shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ShiftChanging, shifttransfer[i]) );
        }
        shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ClutchReleasingSlowly, 0) );
        break;
    }
    case shiftchangingmode::ManualShift: {
        shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ClutchPressing, 0) );
        manualshiftstatusvector shifttransfer = planmanualshiftchangingpartforshifttransfer(manualshiftstatus(aimshift));
        unsigned int transferlength = shifttransfer.size();
        for (unsigned int i = 0; i < transferlength; ++i) {
            shiftchanginglist.push_back( std::make_pair(shiftchangingstatus::ShiftChanging, shifttransfer[i]) );
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

    sectionsusingtime = {0.0, 0.0, 0.0, 0.0, 0.0};

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
        bool ifduringprocess) {
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
            double err = min( percenttolerance * angledistance(shiftpositions[ManualShiftStatus], shiftpositions[aim]), min( angletolerance[1], angletolerance[2] ) );
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

double syscontrolsc::angledistance(std::pair startangles, std::pair stopangles) {
   return sqrt( pow( (startangles.first - stopangles.first), 2 ) + pow( (startangles.second - stopangles.second), 2 ) );
}

syscontrolsc::doublevector syscontrolsc::getinterpshift() {

}

double syscontrolsc::getinterpclutch() {
    const double startPos = clutchpositons[currentclutchindex];
    const double stopPos = clutchpositons[aimclutchindex];
    const double errPos = fabs(startPos - stopPos);

    // 分段线性
    const double speed = curvemotionspeed[0];
    double thresholdround = ceil(0.9 * errPos / speed);

    double pos = startPos + sgn(stopPos - startPos) * round * speed;
    if ((pos - startPos)/(stopPos - startPos) > 1.0) pos = stopPos;
    else if ((pos - startPos)/(stopPos - startPos) > 0.9)
    {
        pos = startPos + sgn(stopPos - startPos) * thresholdround * speed + sgn(stopPos - startPos) * (round - thresholdround) * speed / 2;
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
        feedbackvector.push_back(ShiftChangingStatus);
        feedbackvector.push_back(anglescmd[0]);
        feedbackvector.push_back(anglescmd[1]);
        feedbackvector.push_back(anglescmd[2]);
        feedbackvector.push_back(anglescmd[3]);
        feedbackvector.push_back(anglescmd[4]);
        feedbackvector.push_back(anglescmd[5]);
    }

    if (GearStatus == gearstatus::Auto) {
        feedbackvector.push_back(AutoShiftStatus);
        feedbackvector.push_back(clutchstatus::Released);
    }
    else {
        feedbackvector.push_back(ManualShiftStatus);
        feedbackvector.push_back(ClutchStatus);
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









