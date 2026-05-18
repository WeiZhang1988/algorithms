#include <string>
#include <iostream>
#include <sqlite3.h>

static int callback(void* data, int argc, char** argv, char** azColName) {
   for (int i=0; i<argc; i++) {
      std::cout << azColName[i] << "=" << (argv[i]?argv[i] : "NULL") << " | ";
   }
   std::cout << std::endl;
   return 0;
}
int main()
{
   sqlite3* db;
   char* errMsg = nullptr;
   if (sqlite3_open("test.db", &db) != SQLITE_OK) {
      std::cerr << "open fail: " << sqlite3_errmsg(db) << std::endl;
   }
   const char* sql1 = R"(
   CREATE TABLE IF NOT EXISTS users (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      name TEXT NOT NULL,
      age INTEGER
      );
   )";
   if (sqlite3_exec(db, sql1, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::cerr << "open fail: " << sqlite3_errmsg(db) << std::endl;
   }
   const char* sql2 = R"(
   CREATE TABLE IF NOT EXISTS orders (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      user_id INTEGER,
      product TEXT,
      FOREIGN KEY (user_id) REFERENCES users(id)
      );
   )";
   if (sqlite3_exec(db, sql2, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::cerr << "open fail: " << sqlite3_errmsg(db) << std::endl;
   }
   const char* sql_idx = "CREATE INDEX IF NOT EXISTS idx_user_age ON users(age);";
   if (sqlite3_exec(db, sql_idx, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::cerr << "open fail: " << sqlite3_errmsg(db) << std::endl;
   }
   const char* sql_begin = "BEGIN TRANSACTION;";
   const char* sql_ins_user = R"(
      INSERT INTO users (name, age) VALUES ('MIKE',20), ('KOBE',18);
   )";
   const char* sql_ins_order = R"(
      INSERT INTO orders (user_id, product) VALUES (1,'cellphone'), (2,'computer');
   )";
   const char* sql_commit = "COMMIT;";
   if (sqlite3_exec(db, sql_begin, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::cerr << "open fail: " << sqlite3_errmsg(db) << std::endl;
   }
   if (sqlite3_exec(db, sql_ins_user, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::cerr << "open fail: " << sqlite3_errmsg(db) << std::endl;
   }
   if (sqlite3_exec(db, sql_ins_order, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::cerr << "open fail: " << sqlite3_errmsg(db) << std::endl;
   }
   if (sqlite3_exec(db, sql_commit, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::cerr << "open fail: " << sqlite3_errmsg(db) << std::endl;
   }
   std::cout << "transaction is done. insert user and order" << std::endl << std::endl;
   const char* sql_join = R"(
      SELECT u.name, o.product FROM users u JOIN orders o ON u.id = o.user_id;
   )";
   std::cout << "JOIN search results: " << std::endl;
   sqlite3_exec(db, sql_join, callback, nullptr, &errMsg);
   sqlite3_close(db);
   if(errMsg)
      delete(errMsg);
   return 0;
}
