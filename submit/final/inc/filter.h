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

const float step_filter[] = {0.000663202583744549, 0.00164783478440571, 0.00282257159576915, 0.00398732739623697, 0.00486505550442424, 0.00514020450723020, 0.00451618706224436, 0.00278419813347749, -0.000108015152889177, -0.00400085020714451, -0.00849410351735463, -0.0129549678832616, -0.0165677254769523, -0.0184232683545508, -0.0176392809688452, -0.0134954539140253, -0.00556372783634799, 0.00618773428347678, 0.0213358978965089, 0.0390029137837667, 0.0579199368043738, 0.0765500538259083, 0.0932561667748759, 0.106492004969552, 0.114989964351698, 0.117919045297702, 0.114989964351698, 0.106492004969552, 0.0932561667748759, 0.0765500538259083, 0.0579199368043738, 0.0390029137837667, 0.0213358978965089, 0.00618773428347678, -0.00556372783634799, -0.0134954539140253, -0.0176392809688452, -0.0184232683545508, -0.0165677254769523, -0.0129549678832616, -0.00849410351735463, -0.00400085020714451, -0.000108015152889177, 0.00278419813347749, 0.00451618706224436, 0.00514020450723020, 0.00486505550442424, 0.00398732739623697, 0.00282257159576915, 0.00164783478440571, 0.000663202583744549};

const float hp_filter[] = {0.00637228386546346, 0.00620723550686739, 0.00373996429039832, -0.000630491748576998, -0.00572287357748411, -0.00984946379534327, -0.0113263026648958, -0.00907609451679107, -0.00312702812276919, 0.00518367209119674, 0.0134206999564702, 0.0186733511909672, 0.0184500738681071, 0.0115965651071121, -0.00105207704985454, -0.0165281217223975, -0.0302117342300467, -0.0368730356293289, -0.0320382598842941, -0.0133232947377949, 0.0186449856882842, 0.0600122009064722, 0.104299686973531, 0.143721525388177, 0.170907888690026, 0.180604094410299, 0.170907888690026, 0.143721525388177, 0.104299686973531, 0.0600122009064722, 0.0186449856882842, -0.0133232947377949, -0.0320382598842941, -0.0368730356293289, -0.0302117342300467, -0.0165281217223975, -0.00105207704985454, 0.0115965651071121, 0.0184500738681071, 0.0186733511909672, 0.0134206999564702, 0.00518367209119674, -0.00312702812276919, -0.00907609451679107, -0.0113263026648958, -0.00984946379534327, -0.00572287357748411, -0.000630491748576998, 0.00373996429039832, 0.00620723550686739, 0.00637228386546346};
