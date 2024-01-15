
// /// \brief Publish Measurement object as a marker array. A measurement marker consists of
// ///        a 2D bounding box, its 2D convexhull, a text field about its measurement ID and
// ///        preprocessing debug string, and a upward pointing connecting line from the center of
// ///        the measurement to the text field.
// /// \param publisher
// /// \param header
// /// \param meas_objs
// /// \param color Color and transparency for every marker, helping differentiate different results
// /// \param height_offset Height offset added to every marker, helping differentiate different results
// static void publishMeasurementObjMarkers(const ros::Publisher& publisher, const std_msgs::Header& header,
//                                          const std::vector<ObjectPtr>& meas_objs, const std_msgs::ColorRGBA& color,
//                                          double height_offset = 0.) {
//   // clear all markers before
//   visualization_msgs::MarkerArray obj_markers;
//   visualization_msgs::Marker clear_marker;
//   clear_marker.header = header;
//   clear_marker.ns = "objects";
//   clear_marker.id = 0;
//   clear_marker.action = visualization_msgs::Marker::DELETEALL;
//   clear_marker.lifetime = ros::Duration();
//   obj_markers.markers.push_back(clear_marker);

//   if (meas_objs.empty()) {
//     publisher.publish(obj_markers);
//     return;
//   }

//   visualization_msgs::Marker boxes_marker;
//   int marker_id = 0;
//   boxes_marker.header = header;
//   boxes_marker.ns = "objects";
//   boxes_marker.id = marker_id++;
//   boxes_marker.type = visualization_msgs::Marker::LINE_LIST;
//   boxes_marker.scale.x = 0.02;
//   boxes_marker.color = color;

//   visualization_msgs::Marker polys_marker;
//   polys_marker.header = header;
//   polys_marker.ns = "objects";
//   polys_marker.id = marker_id++;
//   polys_marker.type = visualization_msgs::Marker::LINE_LIST;
//   polys_marker.scale.x = 0.02;
//   polys_marker.color = color;

//   visualization_msgs::Marker id_text;
//   id_text.header = header;
//   id_text.ns = "objects";
//   id_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//   id_text.scale.z = 0.5;
//   id_text.color = color;

//   // connecting sticks between the boxes and the texts
//   visualization_msgs::Marker stick_marker;
//   stick_marker.header = header;
//   stick_marker.ns = "objects";
//   stick_marker.id = marker_id++;
//   stick_marker.type = visualization_msgs::Marker::LINE_LIST;
//   stick_marker.scale.x = 0.05;
//   stick_marker.color = color;

//   geometry_msgs::Point stick[2];
//   geometry_msgs::Point pt;
//   for (size_t obj = 0u; obj < meas_objs.size(); ++obj) {
//     const ObjectPtr& cur_meas = meas_objs[obj];

//     std::vector<geometry_msgs::Point> edges =
//         bogus::getBoxEdges(Eigen::Vector3d(cur_meas->length, cur_meas->width, cur_meas->height), cur_meas->ground_center,
//                            std::atan2(cur_meas->direction[1], cur_meas->direction[0]), true);
//     for (size_t i = 0; i < 8; ++i) {
//       geometry_msgs::Point p = edges[i];
//       p.z += height_offset;
//       boxes_marker.points.push_back(p);
//     }

//     const Eigen::Vector3d center = cur_meas->ground_center + Eigen::Vector3d(0, 0, height_offset);
//     pt.z = center[2] + 0.05;
//     if (!cur_meas->polygon.points.empty()) {
//       for (size_t i = 0; i < cur_meas->polygon.points.size(); i++) {
//         const PolygonDType& polygon = cur_meas->polygon;
//         pt.x = polygon.points[i].x;
//         pt.y = polygon.points[i].y;
//         polys_marker.points.push_back(pt);

//         if (i + 1 >= polygon.points.size()) {
//           pt.x = polygon.points[0].x;
//           pt.y = polygon.points[0].y;
//         } else {
//           pt.x = polygon.points[i + 1].x;
//           pt.y = polygon.points[i + 1].y;
//         }
//         polys_marker.points.push_back(pt);
//       }
//     }

//     stick[0].x = center[0];
//     stick[0].y = center[1];
//     stick[0].z = center[2];
//     stick[1].x = center[0];
//     stick[1].y = center[1];
//     stick[1].z = center[2] + 4.7;
//     stick_marker.points.push_back(stick[0]);
//     stick_marker.points.push_back(stick[1]);

//     std::stringstream ss;
//     if (cur_meas->debug_info.empty()) {
//       ss << "meas: " << cur_meas->id;
//     } else {
//       ss << "meas: " << cur_meas->id << "\n" << cur_meas->debug_info;
//     }
//     id_text.id = marker_id++;
//     id_text.text = ss.str();
//     id_text.pose.position.x = center[0];
//     id_text.pose.position.y = center[1];
//     id_text.pose.position.z = center[2] + 5;

//     switch (cur_meas->type) {
//       case ObjectType::CAR:
//         id_text.color.r = 1.0;
//         id_text.color.g = 0.0;
//         id_text.color.b = 0.0;
//         break;
//       case ObjectType::CYCLIST:
//         id_text.color.r = 0.0;
//         id_text.color.g = 1.0;
//         id_text.color.b = 0.0;
//         break;
//       case ObjectType::PEDESTRIAN:
//         switch (cur_meas->zone_type) {
//           case ZoneType::ROAD:  // cyan
//             id_text.color.r = 0.0;
//             id_text.color.g = 1.0;
//             id_text.color.b = 1.0;
//             break;
//           case ZoneType::ZEBRA_CROSSING:  // megenta
//             id_text.color.r = 1.0;
//             id_text.color.g = 0.0;
//             id_text.color.b = 1.0;
//             break;
//           case ZoneType::DONTCARE:  // blue
//             id_text.color.r = 0.0;
//             id_text.color.g = 0.0;
//             id_text.color.b = 1.0;
//             break;
//           default:
//             break;
//         }
//         break;
//       case ObjectType::TRUCK:
//         id_text.color.r = 1.0;
//         id_text.color.g = 1.0;
//         id_text.color.b = 0.0;
//         break;
//       case ObjectType::DONTCARE:
//         id_text.color.r = 1.0;
//         id_text.color.g = 1.0;
//         id_text.color.b = 1.0;
//         break;
//       default:
//         break;
//     }

//     obj_markers.markers.push_back(id_text);
//   }
//   obj_markers.markers.push_back(boxes_marker);
//   obj_markers.markers.push_back(polys_marker);
//   obj_markers.markers.push_back(stick_marker);

//   publisher.publish(obj_markers);
// }