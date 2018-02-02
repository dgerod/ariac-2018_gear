/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <string>

#include <osrf_gear/ARIAC.hh>

namespace ariac
{
  /// \brief Class to store information about a shipping box.
  class ShippingBox
  {
    /// \brief Constructor.
    public: ShippingBox();

    /// \brief Constructor.
    /// \param[in] _shippingBoxID ID of the shipping box.
    /// \param[in] _assignedShipment Shipment assigned to the shipping box.
    public: ShippingBox(std::string _shippingBoxID);

    /// \brief Destructor.
    public: ~ShippingBox();

    /// \brief Update the current state of the shipment on the shipping box.
    public: void UpdateShipmentState(const Shipment & shipment);

    /// \brief The ID of the shipping box.
    public: std::string shippingBoxID;

    /// \brief The current state of the shipment on the shipping box.
    public: Shipment currentShipment;

    /// \brief Flag for signalling the state of the shipping box has changed.
    protected: bool shipmentStateChanged;
  };
}
