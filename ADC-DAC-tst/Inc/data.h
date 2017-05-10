const uint16_t hr_data[] = {
        94, 96, 99, 102, 105, 108, 111, 113, 114, 115, 114, 114, 112, 111, 109, 107, 105, 103, 101, 100, 
        99, 98, 98, 98, 98, 98, 99, 99, 100, 101, 102, 102, 103, 103, 104, 104, 105, 105, 104, 104, 
        105, 105, 108, 112, 118, 127, 139, 152, 166, 180, 191, 201, 208, 211, 211, 207, 201, 193, 182, 171, 
        160, 149, 138, 128, 118, 110, 104, 98, 94, 92, 91, 91, 92, 94, 97, 100, 103, 106, 109, 111, 
        112, 113, 113, 113, 112, 110, 109, 107, 105, 103, 102, 101, 99, 98, 98, 98, 97, 98, 98, 98, 
        99, 100, 100, 101, 102, 103, 103, 104, 104, 104, 104, 104, 104, 103, 103, 104, 106, 110, 116, 125, 
        136, 149, 164, 178, 190, 201, 208, 211, 211, 208, 202, 194, 183, 172, 161, 149, 138, 128, 119, 110, 
        103, 98, 94, 92, 91, 91, 93, 95, 98, 101, 104, 107, 110, 112, 114, 114, 114, 114, 113, 112, 
        109, 108, 106, 104, 102, 101, 99, 99, 98, 98, 97, 97, 98, 98, 99, 99, 100, 101, 102, 102, 
        103, 103, 103, 104, 103, 103, 103, 103, 103, 105, 107, 111, 117, 126, 138, 151, 165, 179, 191, 200, 
        207, 210, 209, 205, 199, 190, 180, 169, 157, 146, 135, 124, 115, 107, 101, 95, 92, 90, 89, 90, 
        92, 94, 97, 100, 104, 107, 110, 112, 113, 113, 113, 113, 112, 110, 108, 106, 104, 102, 101, 99, 
        98, 97, 96, 96, 96, 96, 97, 97, 98, 99, 100, 100, 101, 102, 102, 103, 103, 104, 103, 104, 
        103, 103, 103, 103, 104, 106, 111, 118, 128, 140, 154, 168, 183, 196, 206, 213, 217, 216, 212, 206, 
        197, 186, 174, 162, 150, 139, 128, 118, 109, 102, 97, 93, 90, 89, 89, 90, 93, 96, 99, 102, 
        105, 108, 110, 111, 112, 113, 112, 111, 110, 109, 107, 105, 103, 102, 100, 99, 98, 97, 97, 96, 
        96, 97, 97, 97, 98, 99, 99, 100, 101, 101, 102, 102, 102, 103, 103, 102, 102, 102, 102, 103, 
        104, 108, 114, 122, 133, 147, 161, 176, 189, 201, 209, 214, 216, 214, 208, 200, 190, 179, 167, 156, 
        144, 133, 123, 114, 106, 99, 94, 90, 89, 88, 89, 91, 93, 96, 100, 103, 106, 108, 110, 111, 
        112, 113, 112, 111, 110, 108, 106, 104, 102, 100, 99, 98, 97, 96, 95, 94, 94, 94, 94, 95, 
        96, 97, 99, 100, 101, 101, 103, 103, 104, 104, 104, 104, 103, 103, 103, 103, 105, 107, 112, 120, 
        130, 143, 157, 172, 186, 198, 207, 213, 214, 213, 208, 201, 191, 180, 168, 156, 144, 132, 122, 113, 
        105, 98, 93, 90, 88, 88, 88, 90, 93, 96, 100, 103, 106, 109, 110, 112, 112, 112, 112, 111, 
        110, 108, 106, 104, 102, 101, 99, 98, 97, 97, 96, 96, 96, 96, 97, 97, 98, 99, 99, 100, 
        101, 101, 101, 102, 102, 103, 103, 103, 102, 103, 102, 103, 104, 107, 112, 120, 130, 142, 156, 171, 
        184, 196, 205, 210, 213, 211, 206, 198, 188, 177, 165, 153, 140, 129, 119, 110, 102, 96, 91, 88, 
        87, 87, 89, 91, 95, 99, 103, 106, 110, 112, 114, 115, 115, 115, 114, 113, 111, 108, 107, 104, 
        102, 101, 99, 98, 97, 97, 96, 96, 96, 97, 97, 98, 98, 99, 100, 101, 101, 102, 102, 102, 
        102, 102, 102, 102, 102, 103, 105, 109, 116, 125, 136, 149, 163, 177, 189, 199, 205, 209, 209, 205, 
        199, 190, 180, 168, 156, 144, 132, 122, 112, 104, 97, 92, 89, 87, 87, 89, 91, 94, 98, 102, 
        105, 108, 111, 113, 115, 115, 115, 114, 113, 112, 110, 108, 106, 104, 102, 100, 99, 98, 98, 97, 
        97, 97, 97, 97, 98, 99, 99, 100, 101, 102, 102, 103, 103, 103, 103, 103, 103, 103, 104, 107, 
        111, 118, 127, 139, 153, 168, 182, 194, 204, 212, 215, 215, 212, 205, 197, 186, 174, 162, 150, 138, 
        128, 118, 109, 102, 96, 92, 90, 89, 90, 91, 94, 97, 100, 104, 107, 110, 112, 114, 114, 115, 
        114, 113, 112, 110, 108, 106, 105, 103, 101, 100, 99, 98, 97, 97, 97, 97, 98, 99, 100, 100, 
        101, 101, 102, 102, 103, 103, 103, 103, 103, 102, 103, 103, 105, 108, 112, 120, 131, 144, 158, 173, 
        186, 198, 207, 213, 215, 214, 210, 203, 194, 183, 171, 160, 148, 136, 126, 116, 108, 101, 96, 93, 
        90, 90, 90, 92, 95, 97, 101, 104, 107, 109, 111, 113, 113, 113, 113, 112, 111, 109, 107, 105, 
        103, 102, 100, 99, 99, 98, 98, 97, 97, 97, 98, 98, 98, 99, 100, 100, 101, 101, 102, 102, 
        103, 103, 103, 103, 103, 103, 104, 105, 107, 112, 119, 129, 141, 155, 169, 182, 195, 204, 210, 213, 
        212, 208, 201, 193, 182, 171, 159, 147, 136, 126, 116, 108, 102, 97, 93, 91, 90, 92, 93, 95, 
        99, 102, 106, 109, 112, 114, 115, 116, 116, 115, 114, 112, 110, 108, 105, 103, 101, 100, 99, 98, 
        98, 97, 97, 98, 98, 98, 99, 100, 101, 101, 102, 103, 103, 104, 104, 104, 104, 104, 104, 104, 
        104, 106, 109, 114, 122, 132, 144, 158, 171, 183, 194, 202, 207, 208, 206, 201, 194, 184, 173, 163, 
        151, 140, 129, 120, 111, 104, 98, 94, 91, 90, 91, 92, 95, 98, 101, 105, 108, 111, 113, 115, 
        116, 116, 115, 114, 113, 111, 109, 107, 105, 103, 101, 100, 99, 98, 98, 98, 98, 98, 99, 99, 
        100, 100, 101, 102, 103, 103, 103, 104, 103, 103, 103, 103, 103, 104, 106, 110, 115, 124, 135, 148, 
        162, 176, 189, 200, 207, 212, 213, 210, 205, 198, 188, 177, 166, 154, 143, 132, 122, 114, 107, 101, 
        97, 94, 92, 91, 91, 93, 95, 98, 101, 104, 107, 109, 111, 113, 113, 113, 113, 111, 110, 108, 
        107, 105, 104, 102, 101, 100, 99, 99, 99, 99, 99, 99, 100, 100, 101, 101, 102, 102, 103, 103, 
};

const float step_filter[] = {0.000439784703912396, 0.000720253593391529, 0.000944949059515744, 0.00100039251117189,
						0.000759709940105169, 0.000107315008841445, -0.00103052510066747, -0.00265382715429144,
						-0.00466156013816171, -0.00683500263486132, -0.00883716627609389, -0.0102317578275973,
						-0.0105223278078980, -0.00920890864562450, -0.00585612077887997, -0.000164013315833560,
						0.00796865425611305, 0.0183990097349776, 0.0307212165769777, 0.0442796578852669,
						0.0582153307665701, 0.0715418235218519, 0.0832427040659712, 0.0923786061826004,
						0.0981903696618233, 0.100184635572996, 0.0981903696618233, 0.0923786061826004,
						0.0832427040659712, 0.0715418235218519, 0.0582153307665701, 0.0442796578852669,
						0.0307212165769777, 0.0183990097349776, 0.00796865425611305, -0.000164013315833560,
						-0.00585612077887997, -0.00920890864562450, -0.0105223278078980, -0.0102317578275973,
						-0.00883716627609389, -0.00683500263486132, -0.00466156013816171, -0.00265382715429144,
						-0.00103052510066747, 0.000107315008841445, 0.000759709940105169, 0.00100039251117189,
						0.000944949059515744, 0.000720253593391529, 0.000439784703912396};
