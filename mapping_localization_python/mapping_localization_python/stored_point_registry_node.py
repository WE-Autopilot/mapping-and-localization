"""In-memory point registry for mapping and planning."""

from copy import deepcopy
from typing import Optional

import rclpy
from ap1_msgs.srv import (
    CreateStoredPoint,
    DeleteStoredPoint,
    LookupStoredPoint,
)
from geometry_msgs.msg import Point
from rclpy.node import Node


class StoredPointRegistryNode(Node):
    """Store geometry points by ID and expose create/delete/lookup services."""

    def __init__(self) -> None:
        super().__init__('stored_point_registry_node')

        self._create_service_name = str(
            self.declare_parameter(
                'create_service_name',
                '/ap1/mapping/point_registry/create',
            ).value
        )
        self._delete_service_name = str(
            self.declare_parameter(
                'delete_service_name',
                '/ap1/mapping/point_registry/delete',
            ).value
        )
        self._lookup_service_name = str(
            self.declare_parameter(
                'lookup_service_name',
                '/ap1/mapping/point_registry/lookup',
            ).value
        )
        self._starting_point_id = int(
            self.declare_parameter('starting_point_id', 1).value
        )

        if self._starting_point_id < 1:
            self.get_logger().warn(
                'starting_point_id must be >= 1; using 1'
            )
            self._starting_point_id = 1

        self._next_point_id = self._starting_point_id
        self._points: dict[int, Point] = {}

        self._create_service = self.create_service(
            CreateStoredPoint,
            self._create_service_name,
            self._handle_create,
        )
        self._delete_service = self.create_service(
            DeleteStoredPoint,
            self._delete_service_name,
            self._handle_delete,
        )
        self._lookup_service = self.create_service(
            LookupStoredPoint,
            self._lookup_service_name,
            self._handle_lookup,
        )

        self.get_logger().info(
            (
                'Stored point registry ready. create: %s, delete: %s, '
                'lookup: %s'
            )
            % (
                self._create_service_name,
                self._delete_service_name,
                self._lookup_service_name,
            )
        )

    def _handle_create(
        self,
        request: CreateStoredPoint.Request,
        response: CreateStoredPoint.Response,
    ) -> CreateStoredPoint.Response:
        point_id = self._next_point_id
        self._next_point_id += 1
        self._points[point_id] = deepcopy(request.point)
        response.id = point_id
        return response

    def _handle_delete(
        self,
        request: DeleteStoredPoint.Request,
        response: DeleteStoredPoint.Response,
    ) -> DeleteStoredPoint.Response:
        response.deleted = request.id in self._points
        if response.deleted:
            del self._points[request.id]
        return response

    def _handle_lookup(
        self,
        request: LookupStoredPoint.Request,
        response: LookupStoredPoint.Response,
    ) -> LookupStoredPoint.Response:
        point = self._points.get(request.id)
        response.found = point is not None
        if point is not None:
            response.point = deepcopy(point)
        return response


def main(args: Optional[list[str]] = None) -> None:
    """Run the stored point registry node."""
    rclpy.init(args=args)
    node = StoredPointRegistryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
