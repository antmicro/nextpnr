#include <vector>
#include "gtest/gtest.h"
#include "nextpnr.h"

USING_NEXTPNR_NAMESPACE

class LP384Test : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
        chipArgs.type = ArchArgs::LP384;
        chipArgs.package = "qn32";
        ctx = new Context(chipArgs);
    }

    virtual void TearDown() { delete ctx; }

    ArchArgs chipArgs;
    Context *ctx;
};

TEST_F(LP384Test, bel_names)
{
    int bel_count = 0;
    for (auto bel : ctx->getBels()) {
        auto name = ctx->getBelName(bel);
        ASSERT_EQ(bel, ctx->getBelByName(name));
        bel_count++;
    }
    ASSERT_EQ(bel_count, 440);
}

TEST_F(LP384Test, wire_names)
{
    int wire_count = 0;
    for (auto wire : ctx->getWires()) {
        auto name = ctx->getWireName(wire);
        assert(wire == ctx->getWireByName(name));
        wire_count++;
    }
    ASSERT_EQ(wire_count, 8294);
}

TEST_F(LP384Test, pip_names)
{
    int pip_count = 0;
    for (auto pip : ctx->getPips()) {
        auto name = ctx->getPipName(pip);
        assert(pip == ctx->getPipByName(name));
        pip_count++;
    }
    ASSERT_EQ(pip_count, 86864);
}

TEST_F(LP384Test, uphill_to_downhill)
{
    for (auto dst : ctx->getWires()) {
        for (auto uphill_pip : ctx->getPipsUphill(dst)) {
            bool found_downhill = false;
            for (auto downhill_pip : ctx->getPipsDownhill(
                         ctx->getPipSrcWire(uphill_pip))) {
                if (uphill_pip == downhill_pip) {
                    ASSERT_FALSE(found_downhill);
                    found_downhill = true;
                }
            }
            ASSERT_TRUE(found_downhill);
        }
    }
}

TEST_F(LP384Test, downhill_to_uphill)
{
    for (auto dst : ctx->getWires()) {
        for (auto downhill_pip : ctx->getPipsDownhill(dst)) {
            bool found_uphill = false;
            for (auto uphill_pip : ctx->getPipsUphill(
                         ctx->getPipDstWire(downhill_pip))) {
                if (uphill_pip == downhill_pip) {
                    ASSERT_FALSE(found_uphill);
                    found_uphill = true;
                }
            }
            ASSERT_TRUE(found_uphill);
        }
    }
}
