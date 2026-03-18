#include "rcore/c_target.h"
#include "lib_hmmd/c_hmmd.h"

#include "cunittest/cunittest.h"

using namespace ncore;

UNITTEST_SUITE_BEGIN(hmmd)
{
    UNITTEST_FIXTURE(read)
    {
        UNITTEST_FIXTURE_SETUP() 
		{
            nsensors::initHMMD(16, 17);

		}
        UNITTEST_FIXTURE_TEARDOWN() {}

        UNITTEST_TEST(test_1)
        {
            s8  ok       = 0;
            u16 distance = 0;
            nsensors::readHMMD(&ok, &distance);
        }
    }

    UNITTEST_FIXTURE(read2)
    {
        UNITTEST_TEST(test_1) 
		{
            s8  ok       = 0;
            u16 distance = 0;
            CHECK_TRUE (nsensors::readHMMD2(&ok, &distance));
            {
                CHECK_EQUAL(1, ok);
                CHECK_EQUAL(60, distance);
            }
		}
    }
}
UNITTEST_SUITE_END
