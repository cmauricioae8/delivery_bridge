import rclpy
import uvicorn
from threading import Thread

from delivery_bridge.base_node import base_node
from delivery_bridge.webapp.main import app as webapp
from delivery_bridge.webapp.main import sio
# from delivery_bridge_bridge.modules.function_manager import function_manager

from delivery_bridge.webapp.database import createDatabase
from delivery_bridge.webapp.apps.users.cruds.user_cruds import user_crud
from delivery_bridge.webapp.apps.users.models import User


def main(args=None):
    ################################################################################
    # Create the database if it doesn't exist
    db_success = createDatabase()
    if db_success:
        base_node.logger.info("Database created successfully.")
    else:
        base_node.logger.error("Database creation failed.")

    # Create the default admin user if it doesn't exist
    users: list[User] = user_crud.get_by_field("is_admin", True, allows_multiple=True)
    if len(users) == 0:
        base_node.logger.info("No admin user found, creating default user")
        user = User(username="admin", is_admin=True)
        user.set_password("admin")
        user.save()
        base_node.logger.info("Default user created")

    
    # Start the webapp
    webapp_thread = Thread(
        target=uvicorn.run, kwargs={"app": webapp, "host": "0.0.0.0", "port": 9009}
    )
    webapp_thread.daemon = True
    webapp_thread.start()


    base_node.init_topics()

    # register sio events
    @sio.event
    async def cmd_vel(sid, data):
        linear_x = float(data["linear_x"])
        angular_z = float(data["angular_z"])
        # logger.info("robot_move \tx:{:2.2f}, \tz:{:2.2f}".format(linear_x,angular_z))
        base_node.cmd_vel_publisher.publish(linear_x, angular_z)

    rclpy.spin(base_node)

    base_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
