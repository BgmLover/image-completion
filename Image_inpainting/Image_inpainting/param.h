#pragma once

#define KS 50
#define KI 2
#define PatchSizeRow 12
#define PatchSizeCol 12

typedef enum {
	INNER,BORDER,OUTER
}PointType;

//control the debug
#define ifshowDPlabel true
#define ifshowDP_M	  true
#define ifshowDP_E1   true

#define ifshowBPlabel true
#define ifshowBP_E1   true
#define ifshowCurves  true
#define ifshowsrcImg  false