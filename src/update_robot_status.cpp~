#include <stdlib.h>
#include <iostream>
#include <string>

#include "mysql_connection.h"

#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>

#include <ros/ros.h>
#include <std_msgs/Int16.h>

using namespace std;

// Database variables
long userId = 1;
long const Sn = 1;

int batteryLevel;
int waterStatus;
int dustLevel;

void batteryLevelCallback(const std_msgs::Int16 &batteryLevelSTD)
{
	batteryLevel = batteryLevelSTD.data;

} // end batteryLevelCallback

void waterStatusCallback (const std_msgs::Int16 &waterStatusSTD)
{
	waterStatus = waterStatusSTD.data;

} // end waterStatusCallback

void dustLevelCallback (const std_msgs::Int16 &dustLevelSTD)
{
	dustLevel = dustLevelSTD.data;

} // end dustLevelCallback

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dbNode");
	ros::NodeHandle nodeHandle;

	ros::Subscriber batteryLevelSubscriber = nodeHandle.subscribe("batteryLevelTopic", 10, batteryLevelCallback);
	ros::Subscriber waterStatusSubscriber = nodeHandle.subscribe("waterLevelTopic", 10, waterStatusCallback);
	ros::Subscriber dustLevelSubscriber = nodeHandle.subscribe("dustLevelTopic", 10, dustLevelCallback);

	ros::Rate r(1.0);

	while(nodeHandle.ok())
	{
		ros::spinOnce();
		try
		{
			sql::Driver *driver;
			sql::Connection *connection;
			//sql::Statement *statement;
			sql::PreparedStatement *preparedStatement;
			//sql::ResultSet *resultSet;

			driver = get_driver_instance();
			connection = driver->connect("tcp://52.39.162.195:3306", "root", "12345678");
			connection->setSchema("cleaningrobot");
			// Prepare values for SQl query

			//todo: fix the waterStatus variables issue: space?
			if(waterStatus== 0)
				preparedStatement = connection->prepareStatement("UPDATE Robot SET batteryLevel=?, waterStatus='Clean', dustLevel=? WHERE userId=? AND Sn=?;");
			else
				preparedStatement = connection->prepareStatement("UPDATE Robot SET batteryLevel=?, waterStatus='Needs emptying', dustLevel=? WHERE userId=? AND Sn=?;");

			preparedStatement->setInt(1, batteryLevel);
			// preparedStatement->setString(2, waterStatus);
			preparedStatement->setInt(2, dustLevel);
			preparedStatement->setInt(3, userId);
			preparedStatement->setInt(4, Sn);

			preparedStatement->execute();

			//delete resultSet;
			delete preparedStatement;
			//delete statement;
			delete connection; 

		} // end try

		catch(sql::SQLException &e)
		{
			cout << "# ERR: " << e.what();
  			cout << " (MySQL error code: " << e.getErrorCode();
			cout << ", SQLState: " << e.getSQLState() << " )" << endl;

		} // end catch

		r.sleep();

	} // end while

} // end main
