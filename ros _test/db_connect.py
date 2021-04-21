import MySQLdb as mysql


def write_db():
    con = mysql.connect(host='localhost', user='root', passwd='passwd', db='test')
    cursor = con.cursor()
    cursor.execute('select * from test;')
    res = cursor.fetchall()
    print (res)
    con.close()


if __name__ == '__main__':
    con = mysql.connect(host='localhost', user='root', passwd='passwd', db='test')
    cursor = con.cursor()
    cursor.execute('show tables;')
    res = cursor.fetchall()
    print (res)
    cursor.execute('insert into test values(2, "mingming");')
    cursor.execute('select * from test;')
    res = cursor.fetchall()
    print (res)
    con.close()
