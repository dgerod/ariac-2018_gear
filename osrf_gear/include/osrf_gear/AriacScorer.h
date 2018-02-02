/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
/*
 * Desc: ARIAC scorer.
 * Author: Deanna Hood
 */
#ifndef _ROS_ARIAC_SCORER_HH_
#define _ROS_ARIAC_SCORER_HH_

#include <map>
#include <string>

#include <ros/ros.h>

#include "osrf_gear/ARIAC.hh"
#include "osrf_gear/AriacShippingBox.h"
#include <osrf_gear/ShippingBox.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/ShippingBoxContents.h>
#include "osrf_gear/VacuumGripperState.h"

/// \brief A scorer for the ARIAC game.
class AriacScorer
{
  /// \brief Constructor.
  public: AriacScorer();

  /// \brief Destructor.
  public: virtual ~AriacScorer();

  /// \brief Update the scorer.
  public: void Update(double timeStep = 0.0);

  /// \brief Get the current score.
  /// \param[in] The time in seconds since the last update.
  /// \return The score for the game.
  public: ariac::GameScore GetGameScore();

  /// \brief Get the score of the current order.
  /// \return True if the order is complete.
  public: bool IsOrderComplete(const ariac::OrderID_t & orderID);

  /// \brief Get the score of the current order.
  /// \return The score for the order.
  public: ariac::OrderScore GetOrderScore(const ariac::OrderID_t & orderID);

  /// \brief Assign an order to process.
  /// \param[in] order The order.
  public: void AssignOrder(const ariac::Order & order);

  /// \brief Stop processing the current order.
  /// \param[in] timeTaken The time spent on the order.
  /// \return The score for the order.
  public: ariac::OrderScore UnassignOrder(const ariac::OrderID_t & orderID);

  /// \brief Get the shipping boxes the scorer is monitoring.
  /// \return Vector of shipping box states.
  public: std::vector<ariac::ShippingBox> GetShippingBoxes();

  /// \brief Get the shipping box with the specified ID.
  /// \param[in] shippingBoxID The ID of the shipping box to get.
  /// \param[in] shippingBox The shippingBox found.
  /// \return True if the shipping box was found, false otherwise.
  public: bool GetShippingBoxById(const ariac::ShippingBoxID_t & shippingBoxID, ariac::ShippingBox & shippingBox);

  /// \brief Submit shipping box for scoring and store the result in the order score.
  public: ariac::ShipmentScore SubmitShipment(const ariac::ShippingBox & shippingBox);

  /// \brief Calculate the score for a shipping box given the type of shipment being built.
  protected: ariac::ShipmentScore ScoreShippingBox(const ariac::ShippingBox & shippingBox, const ariac::Shipment & assignedShipment);

  /// \brief Helper function for filling a Shipment from a shipping box contents ROS message.
  public: static void FillShipmentFromMsg(const osrf_gear::ShippingBoxContents::ConstPtr & shippingBoxMsg, ariac::Shipment & shipment);

  /// \brief Helper function for filling a Shipment from a shipment ROS message.
  public: static void FillShipmentFromMsg(const osrf_gear::Shipment & shipmentMsg, ariac::Shipment & shipment);

  /// \brief Callback for receiving order message.
  public: void OnOrderReceived(const osrf_gear::Order::ConstPtr & orderMsg);

  /// \brief Callback for receiving shipping box state message.
  public: void OnShippingBoxInfoReceived(const osrf_gear::ShippingBoxContents::ConstPtr & shippingBoxMsg);

  /// \brief Callback for receiving gripper state message.
  public: void OnGripperStateReceived(const osrf_gear::VacuumGripperState &stateMsg);

  /// \brief The shipping boxes to monitor the score of.
  protected: std::map<ariac::ShippingBoxID_t, ariac::ShippingBox> shippingBoxes;

  /// \brief Mutex for protecting the orders being scored.
  protected: mutable boost::mutex mutex;

  /// \brief Collection of orders that have been announced but are not yet complete.
  protected: std::vector<ariac::Order> ordersInProgress;

  /// \brief Flag for signalling new shipping box info to process.
  protected: bool newShippingBoxInfoReceived = false;

  /// \brief Flag for signalling new order to process.
  protected: bool newOrderReceived = false;

  /// \brief Whether or not there is a travelling part in the gripper.
  protected: bool isProductTravelling = false;

  /// \brief Order receivd from order messages.
  protected: ariac::Order newOrder;

  /// \brief Parameters to use for calculating scores.
  protected: ariac::ScoringParameters scoringParameters;

  /// \brief Pointer to the score of the current order.
  protected: ariac::OrderScore* orderScore;

  /// \brief The score of the run.
  protected: ariac::GameScore gameScore;

};
#endif
