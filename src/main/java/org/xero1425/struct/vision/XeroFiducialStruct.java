package org.xero1425.struct.vision;

import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;

public class XeroFiducialStruct implements Struct<XeroFiducial> {

    @Override
    public Class<XeroFiducial> getTypeClass() {
        return XeroFiducial.class;
    }

    @Override
    public String getTypeString() {
        return "struct:XeroFiducial";
    }

    @Override
    public String getSchema() {
        return "double id;double area;double x;double y";
    }

    @Override
    public int getSize() {
        return kSizeDouble * 4;
    }

    @Override
    public void pack(ByteBuffer bb, XeroFiducial value) {
        bb.putDouble(value.id);
        bb.putDouble(value.area);
        bb.putDouble(value.x);
        bb.putDouble(value.y);
    }

    @Override
    public XeroFiducial unpack(ByteBuffer bb) {
        return new XeroFiducial(
            bb.getDouble(),
            bb.getDouble(),
            bb.getDouble(),
            bb.getDouble()
        );
    }
    
}