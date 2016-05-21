/* Standard C++ includes */
#include <stdlib.h>
#include <iostream>

#include "mysql_connection.h"

#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>

using namespace std;

int robotSn = 1;

struct Task
{
  long Id;
  long robotSn;
  string dueDate;
  string startingTime;
  string operation; 

}; // end struct Report

int main(void)
{

Task *tasks;
Task earliestTask;
int reportsNo = 0;

try
 {
  sql::Driver *driver;
  sql::Connection *connection;
  //sql::Statement *statement;
  sql::ResultSet *resultSet;
  sql::PreparedStatement *preparedStatement;

  /* Create a connection */
  driver = get_driver_instance();
  connection = driver->connect("tcp://52.39.162.195:3306", "root", "12345678");
  /* Connect to the MySQL database */
  connection->setSchema("cleaningrobot");


  preparedStatement = connection->prepareStatement("SELECT * FROM Task WHERE robotSn = ?");
  preparedStatement->setInt(1, robotSn);
  resultSet = preparedStatement->executeQuery();

  reportsNo = resultSet->rowsCount();

  tasks = new Task [100];

  int count = 0;
  while(resultSet->next())
  {
    tasks[count].Id = resultSet->getInt(1);
    tasks[count].robotSn = resultSet->getInt(2);
    tasks[count].dueDate = resultSet->getString(3);
    tasks[count].startingTime = resultSet->getString(4);
    tasks[count].operation = resultSet->getString(5);


    count ++;

  } // end while

  reportsNo = count;

  cout << "Count is: " << count << endl;
  cout << "Reports Number is: " << reportsNo << endl;

  delete preparedStatement;
  delete resultSet;
  delete connection;
  
 } // end try
  catch (sql::SQLException &e)
  {
   cout << "# ERR: " << e.what();
   cout << " (MySQL error code: " << e.getErrorCode();
   cout << ", SQLState: " << e.getSQLState() << " )" << endl;

  } // end catch

  // objective: get the earliest task
  // assume that it is the one with the largest id!! should be changed

  earliestTask = tasks[0];
  for (int i = 0; i < reportsNo; i++)
  {
    if(tasks[i].Id > earliestTask.Id)
    {
       earliestTask = tasks[i];

    } // end if

    cout << "Task id is: " << tasks[i].Id << endl;

  } // end for

  cout << "Earliest task Id is: " << earliestTask.Id << endl;
  cout << "Earliest task due date is: " << earliestTask.dueDate << endl;
  cout << "Earliest task starting time is: " << earliestTask.startingTime << endl;
  cout << "Earliest task operation is: " << earliestTask.operation << endl;

  // construct the system command in the form:
    // at hh:mm PM Month dd yyyy <<< "./script.sh"
    // e.g. at 4:35 PM May 20 2016 <<< "./ros_launch.sh"

  string command = "at " + earliestTask.startingTime + " " + earliestTask.dueDate 
       + " < \"./dash_script.sh\"";


  cout << command.c_str() << endl;
  system(command.c_str());


return 0;

} // end main
