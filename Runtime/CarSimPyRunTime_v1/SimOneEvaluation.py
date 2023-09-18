from SimOneIOStruct import *

InitEvaluationService = SimoneAPI.InitEvaluationService
InitEvaluationServiceWithLocalData = SimoneAPI.InitEvaluationServiceWithLocalData
AddEvaluationRecord = SimoneAPI.AddEvaluationRecord
SaveEvaluationRecord = SimoneAPI.SaveEvaluationRecord


EvaluationCbFuncType = CFUNCTYPE(c_void_p, POINTER(SimOne_Data_JudgeEvent))
SIMONEAPI_EVALUATION_CB = None


Evaluation_CB=None

def _simoneapi_evaluation_cb(data):
	global SIMONEAPI_EVALUATION_CB
	SIMONEAPI_EVALUATION_CB(data)

simoneapi_evaluation_cb_func=EvaluationCbFuncType(_simoneapi_evaluation_cb)


def SoGetJudgeData(mainvechileId,data):
	pass

def SoGetJudgeEventCB(cb):
	global Evaluation_CB
	SimoneAPI.SetJudgeEventCB(simoneapi_evaluation_cb_func)
	Evaluation_CB=cb

