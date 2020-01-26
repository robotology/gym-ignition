/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_CONTROLLERS_POSITIONCONTROLLER_H
#define GYMPP_CONTROLLERS_POSITIONCONTROLLER_H

namespace gympp {
    class Robot;
    namespace controllers {
        class PositionController;
        class PositionControllerReferences;
    } // namespace controllers
} // namespace gympp

class gympp::controllers::PositionController
{
public:
    virtual ~PositionController() = default;
    virtual bool
    setReferences(const gympp::controllers::PositionControllerReferences& references) = 0;
};

#endif // GYMPP_CONTROLLERS_POSITIONCONTROLLER_H
