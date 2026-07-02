class FormatTestTmp
{
public:
    FormatTestTmp(int value);
    int getValue();
    void setValue(int v);
private:
    int value;
};

FormatTestTmp::FormatTestTmp( int value )
{
	this->value = value;
}

int FormatTestTmp::getValue( )
{
    if(value>0){
		return value;
	}
	else
	{
        return   0;
	}
}

void FormatTestTmp::setValue(int   v)
{
	value=v;
}
