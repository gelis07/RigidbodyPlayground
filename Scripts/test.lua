p = 
{
    MoveStep = 170,
    JumpVel = 200
}

function Update()
    if Input("LEFT") then
        SetVelocity(-p.MoveStep, VelocityY())
    elseif Input("RIGHT") then
        SetVelocity(p.MoveStep, VelocityY())
    elseif Input("UP") then
        SetVelocity(VelocityX(), VelocityY() + p.JumpVel)
    end
end