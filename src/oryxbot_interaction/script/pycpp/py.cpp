#include<iostream>
#include<Python.h>
using namespace std;
int main(int argc, char* argv[])
{
    //初始化python
    Py_Initialize();

    //直接运行python代码
    PyRun_SimpleString("print('----------Python Start')");

    //PyImport_Import("rospy");

    //引入当前路径,否则下面模块不能正常导入
    PyRun_SimpleString("import sys");
    //PyRun_SimpleString("import rospy");  
    PyRun_SimpleString("sys.path.append('./')");  


    // PyRun_SimpleString("import rospy");  
    // PyRun_SimpleString("from oryxbot_msgs.srv import *");  

    //引入模块
    PyRun_SimpleString("print('----------PyImport_ImportModule')");
    PyObject *pModule = PyImport_ImportModule("oryxbot_server1");
    //获取模块字典属性
    PyRun_SimpleString("print('----------PyModule_GetDict')");
    PyObject *pDict = PyModule_GetDict(pModule);

    //直接获取模块中的函数
    PyRun_SimpleString("print('----------PyObject_GetAttrString')");
    PyObject *pFunc = PyObject_GetAttrString(pModule, "open_camera");

    //参数类型转换，传递一个字符串。将c/c++类型的字符串转换为python类型，元组中的python类型查看python文档
    PyRun_SimpleString("print('----------Py_BuildValue')");
    PyObject *pArg = Py_BuildValue("(s)", "Hello Charity");

    PyRun_SimpleString("print('----------PyEval_CallObject')");
    //调用直接获得的函数，并传递参数
    PyEval_CallObject(pFunc, pArg);

    //释放python
    Py_Finalize();
    getchar();
    return 0;
}
