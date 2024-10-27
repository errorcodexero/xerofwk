package org.xero1425.struct.vision;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;

public class XeroPoseEstimateStruct implements Struct<XeroPoseEstimate> {

    @Override
    public String getSchema() {
        return "Pose2d pose; double timestamp; int32 tagCount; bool valid";
    }

    @Override
    public int getSize() {
        return Pose2d.struct.getSize() + kSizeDouble + kSizeInt32 + kSizeBool;
    }

    @Override
    public Class<XeroPoseEstimate> getTypeClass() {
        return XeroPoseEstimate.class;
    }

    @Override
    public String getTypeString() {
        return "struct:XeroPoseEstimate";
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct<?>[] {Pose2d.struct};
    }

    @Override
    public void pack(ByteBuffer bb, XeroPoseEstimate value) {
        Pose2d.struct.pack(bb, value.pose);
        bb.putDouble(value.timestamp);
        bb.putInt(value.tagCount);
        bb.put((byte) (value.valid ? 1 : 0));
    }

    @Override
    public XeroPoseEstimate unpack(ByteBuffer bb) {
        return new XeroPoseEstimate(Pose2d.struct.unpack(bb), bb.getDouble(), bb.getInt(), bb.get() == 1 ? true : false);
    }
   
}