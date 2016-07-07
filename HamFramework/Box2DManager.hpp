//-----------------------------------------------
//
//	This file is part of the HamFramework for Siv3D.
//
//	Copyright (C) 2014-2016 Hamukun
//	Copyright (C) 2014-2016 Ryo Suzuki
//
//	Licensed under the MIT License.
//
//-----------------------------------------------

# pragma once
# include <Siv3D.hpp>
# include <Siv3DAddon\Box2D.hpp>

namespace ham
{
	constexpr Vec2 ToVec2(const b2Vec2& b2Vec2)
	{
		return{ b2Vec2.x, b2Vec2.y };
	}

	b2Vec2 ToB2Vec2(const Vec2& vec2)
	{
		return{ static_cast<float32>(vec2.x), static_cast<float32>(vec2.y) };
	}

	class Box2DBody;
	using PBody = std::shared_ptr<Box2DBody>;

	struct Box2DStatus
	{
		double density = 1.0;

		double restitution = 0.0;

		double friction = 0.2;

		constexpr Box2DStatus(double _density = 1.0, double _restitution = 0.1, double _friction = 0.2)
			: density(_density)
			, restitution(_restitution)
			, friction(_friction) {}
	};

	class Box2DShape
	{
	protected:

		b2FixtureDef m_fixtureDef;

		Color m_color = Palette::White;

	public:

		using Box2DBodyPtr = std::weak_ptr<Box2DBody>;

		Box2DShape(const b2Shape* shape)
		{
			m_fixtureDef.shape = shape;
		}

		void setDensity(double density)
		{
			m_fixtureDef.density = static_cast<float32>(density);
		}

		void setFriction(double friction)
		{
			m_fixtureDef.friction = static_cast<float32>(friction);
		}

		void setRestitution(double restitution)
		{
			m_fixtureDef.restitution = static_cast<float32>(restitution);
		}

		const b2FixtureDef& getFixtureDef() const
		{
			return m_fixtureDef;
		}

		void setColor(const Color& color)
		{
			m_color = color;
		}

		void setFilterGroupIndex(const int16 groupIndex)
		{
			m_fixtureDef.filter.groupIndex = static_cast<int16>(groupIndex);
		}

		virtual void draw(const Box2DBodyPtr& body) const = 0;
	};

	class Box2DInitialBodyStatus
	{
	private:

		b2BodyDef m_bodyDef;

	public:

		void setBodyType(b2BodyType type)
		{
			m_bodyDef.type = type;
		}

		virtual void setPos(const Vec2& pos)
		{
			m_bodyDef.position = ToB2Vec2(pos);
		}

		void setAngularDamping(double anglarDamping)
		{
			m_bodyDef.angularDamping = static_cast<float32>(anglarDamping);
		}

		void setAngle(const double angle)
		{
			m_bodyDef.angle = static_cast<float32>(angle);
		}

		void setBullet(const bool bullet)
		{
			m_bodyDef.bullet = bullet;
		}

		const b2BodyDef& getBodyDefinition() const
		{
			return m_bodyDef;
		}
	};

	class Box2DBody : public std::enable_shared_from_this<Box2DBody>
	{
	public:

		using WorldWeakPtr = std::weak_ptr<b2World>;
		using BodyPtr = b2Body*;
		using InitialBodyStatusPtr = std::shared_ptr<Box2DInitialBodyStatus>;
		using ShapePtr = std::shared_ptr<Box2DShape>;
		using Box2DBodyPtr = std::weak_ptr<Box2DBody>;

		Box2DBody(const WorldWeakPtr& world)
			: m_world(world)
			, m_initialBodyStatus(std::make_shared<Box2DInitialBodyStatus>()) {}

		void init(b2BodyType bodyType, const Vec2& center = Vec2(0, 0))
		{
			auto initialStatus = std::make_shared<Box2DInitialBodyStatus>();
			initialStatus->setPos(center);
			initialStatus->setBodyType(bodyType);
			m_initialBodyStatus = initialStatus;

			myPtr = shared_from_this();

			createBody();
		}

		virtual ~Box2DBody()
		{
			if (!m_world.expired())
			{
				m_world.lock()->DestroyBody(m_body);
			}
		}

		const BodyPtr& getBodyPtr() const
		{
			return m_body;
		}

		virtual void draw() const
		{
			for (const auto& fixtureDef : m_fixtureDefinitions)
			{
				fixtureDef->draw(myPtr);
			}
		}

		void setShape(const ShapePtr& shape)
		{
			m_fixtureDefinitions.push_back(shape);
		}

		void setShape(const Line& line, const Box2DStatus& status = Box2DStatus());

		void setShape(const LineString& line, const Box2DStatus& status = Box2DStatus());

		void setShape(const RectF& rect, const Box2DStatus& status = Box2DStatus());

		void setShape(const Circle& circle, const Box2DStatus& status = Box2DStatus());

		void setShape(const Polygon& polygon, const Box2DStatus& status = Box2DStatus());

		void setPos(const double x, const double y)
		{
			m_body->SetTransform(b2Vec2(static_cast<float32>(x), static_cast<float32>(y)), m_body->GetAngle());
		}

		void setPos(const Vec2& pos)
		{
			setPos(pos.x, pos.y);
		}

		void setAngle(const double angle)
		{
			m_body->SetTransform(m_body->GetPosition(), static_cast<float32>(angle));
		}

		void setTransform(const double x, const double y, const double angle)
		{
			m_body->SetTransform(b2Vec2(static_cast<float32>(x), static_cast<float32>(y)), static_cast<float32>(angle));
		}

		void setTransform(const Vec2& pos, const double angle)
		{
			setTransform(pos.x, pos.y, angle);
		}

		void setAwake(const bool awake)
		{
			m_body->SetAwake(awake);
		}

		const InitialBodyStatusPtr& getInitialBodyStatusPtr() const
		{
			return m_initialBodyStatus;
		}

		void setInitialBodyStatusPtr(const InitialBodyStatusPtr& initialBodyStatus)
		{
			m_initialBodyStatus = initialBodyStatus;
		}

		Vec2 getPos() const
		{
			return ToVec2(m_body->GetPosition());
		}

		double getAngle() const
		{
			return static_cast<double>(m_body->GetAngle());
		}

	protected:

		virtual void createBody()
		{
			m_body = m_world.lock()->CreateBody(&m_initialBodyStatus->getBodyDefinition());

			for (const auto& fixtureDef : m_fixtureDefinitions)
			{
				m_body->CreateFixture(&fixtureDef->getFixtureDef());
			}
		}

		Box2DBodyPtr myPtr;

		WorldWeakPtr m_world;

		BodyPtr m_body;

		InitialBodyStatusPtr m_initialBodyStatus;

		Array<ShapePtr> m_fixtureDefinitions;
	};

	class Box2DLine : public Box2DShape
	{
	private:

		b2EdgeShape m_edgeShape;

		Line m_line;

	public:

		Box2DLine()
			: Box2DShape(&m_edgeShape) {}

		void setLine(const Line& line)
		{
			m_edgeShape.Set(ToB2Vec2(line.begin), ToB2Vec2(line.end));

			m_line = line;
		}

		void draw(const Box2DBodyPtr& body) const override
		{
			const double angle = body.lock()->getBodyPtr()->GetAngle();

			Line(m_line.begin.rotated(angle), m_line.end.rotated(angle))
				.moveBy(ToVec2(body.lock()->getBodyPtr()->GetPosition())).draw(0.1, m_color);
		}
	};

	class Box2DLineString : public Box2DShape
	{
	private:

		b2ChainShape m_chainShape;

		LineString m_lineString;

		bool m_isClosedCurve = false;

	public:

		Box2DLineString()
			: Box2DShape(&m_chainShape) {}

		void draw(const Box2DBodyPtr& body) const override
		{
			const double angle = body.lock()->getBodyPtr()->GetAngle();
			const Vec2 pos = ToVec2(body.lock()->getBodyPtr()->GetPosition());

			Array<Vec2> points;

			for (const auto& point : m_lineString.getArray())
			{
				points.push_back(pos + point.rotated(angle));
			}

			LineString(points).draw(0.1, m_color, m_isClosedCurve);
		}

		void setLineStringOpen(const LineString& lineString)
		{
			Array<b2Vec2> points;

			for (const auto& point : lineString)
			{
				points.push_back(ToB2Vec2(point));
			}

			m_lineString = lineString;

			m_chainShape.CreateChain(points.data(), points.size());

			m_isClosedCurve = false;
		}

		void setLineStringLoop(const LineString& lineString)
		{
			Array<b2Vec2> points;

			for (const auto& point : lineString)
			{
				points.push_back(ToB2Vec2(point));
			}

			m_lineString = lineString;

			m_chainShape.CreateLoop(points.data(), points.size());

			m_isClosedCurve = true;
		}
	};

	class Box2DRect : public Box2DShape
	{
	private:

		b2PolygonShape m_polygonShape;

		float32 width;

		float32 height;

	public:

		Box2DRect()
			: Box2DShape(&m_polygonShape) {}

		void draw(const Box2DBodyPtr& body) const override
		{
			const double angle = body.lock()->getBodyPtr()->GetAngle();

			const Vec2 pos = ToVec2(body.lock()->getBodyPtr()->GetPosition());

			RectF(width, height).setCenter(pos).rotated(angle).draw(m_color);
		}

		void setRect(const RectF& rect)
		{
			width = static_cast<float32>(rect.w);
			height = static_cast<float32>(rect.h);
			m_polygonShape.SetAsBox(width * 0.5f, height * 0.5f, ToB2Vec2(rect.center), 0.0f);
		}
	};

	class Box2DCircle : public Box2DShape
	{
	private:

		b2CircleShape m_circleShape;

	public:

		Box2DCircle()
			: Box2DShape(&m_circleShape) {}

		void draw(const Box2DBodyPtr& body) const override
		{
			const double angle = body.lock()->getBodyPtr()->GetAngle();

			const Vec2 pos = ToVec2(body.lock()->getBodyPtr()->GetPosition());

			Circle(pos, m_circleShape.m_radius).draw(m_color);
		}

		void setRadius(double radius)
		{
			m_circleShape.m_radius = static_cast<float32>(radius);
		}

		void setPosition(const Vec2& pos)
		{
			m_circleShape.m_p = ToB2Vec2(pos);
		}
	};

	class Box2DPolygon : public Box2DShape
	{
	private:

		b2PolygonShape m_polygonShape;

		Polygon m_polygon;

	public:

		Box2DPolygon()
			: Box2DShape(&m_polygonShape) {}

		void setPolygon(const Polygon& polygon)
		{
			Array<b2Vec2> vertices;

			for (const auto& point : polygon.outer())
			{
				vertices.push_back(ToB2Vec2(point));
			}

			m_polygonShape.Set(vertices.data(), vertices.size());

			m_polygon = polygon;
		}

		void draw(const Box2DBodyPtr& body) const override
		{
			const double angle = body.lock()->getBodyPtr()->GetAngle();

			const Vec2 pos = ToVec2(body.lock()->getBodyPtr()->GetPosition());

			m_polygon.rotated(angle).movedBy(pos).draw(m_color);
		}
	};

	struct Box2DWheelJointStatus
	{
		Vec2 localAnchorA;

		Vec2 localAnchorB;

		Vec2 localAxisA;

		double frequencyHz;

		double dampingRatio;

		double motorSpeed;

		double maxMotorTorque;

		constexpr Box2DWheelJointStatus(const Vec2& _localAnchorA = Vec2::Zero
			, const Vec2& _localAnchorB = Vec2::Zero
			, const Vec2& _localAxisA = Vec2(1.0, 0.0)
			, double _frequencyHz = 2.0
			, double _dampingRatio = 0.7
			, double _motorSpeed = 0.0
			, double _maxMotorTorque = 0.0)
			: localAnchorA(_localAnchorA)
			, localAnchorB(_localAnchorB)
			, localAxisA(_localAxisA)
			, frequencyHz(_frequencyHz)
			, dampingRatio(_dampingRatio)
			, motorSpeed(_motorSpeed)
			, maxMotorTorque(_maxMotorTorque)
		{}
	};

	class Box2DInitialWheelJointStatus
	{
	public:

		using WheelJointDef = b2WheelJointDef;
		using Box2DBodyPtr = std::shared_ptr<Box2DBody>;

		void setMotorSpeed(double motorSpeed)
		{
			m_wheelJointDef.motorSpeed = static_cast<float32>(motorSpeed);
		}

		void setMaxMotorTorque(double maxMotorTorque)
		{
			m_wheelJointDef.maxMotorTorque = static_cast<float32>(maxMotorTorque);
		}

		void setFrequencyHz(double frequencyHz)
		{
			m_wheelJointDef.frequencyHz = static_cast<float32>(frequencyHz);
		}

		void setDampingRatio(double dampingRatio)
		{
			m_wheelJointDef.dampingRatio = static_cast<float32>(dampingRatio);
		}

		void enableMotor(bool flag)
		{
			m_wheelJointDef.enableMotor = flag;
		}

		void init(const Box2DBodyPtr& bodyA, const Box2DBodyPtr& bodyB, const Vec2& anchor, const Vec2& axis)
		{
			m_wheelJointDef.Initialize(bodyA->getBodyPtr(), bodyB->getBodyPtr(), ToB2Vec2(anchor), ToB2Vec2(axis));
		}

		const WheelJointDef& getWheelJointDefinition() const
		{
			return m_wheelJointDef;
		}

	private:

		WheelJointDef m_wheelJointDef;

	};

	class Box2DWheelJoint
	{
	public:

		using WorldWeakPtr = std::weak_ptr<b2World>;
		using InitialWheelJointStatusPtr = std::shared_ptr<Box2DInitialWheelJointStatus>;
		using WheelJointPtr = b2WheelJoint*;


		Box2DWheelJoint(const WorldWeakPtr& world)
			: m_world(world)
		{

		}

		virtual ~Box2DWheelJoint()
		{
			if (!m_world.expired())
				m_world.lock()->DestroyJoint(m_wheelJoint);
		}

		void setInitialWheelJointStatus(const InitialWheelJointStatusPtr& initialWheelJointStatus)
		{
			m_initialWheelJointStatus = initialWheelJointStatus;
		}

		void setMotorSpeed(double motorSpeed)
		{
			m_wheelJoint->SetMotorSpeed(static_cast<float32>(motorSpeed));
		}

		void init()
		{
			m_wheelJoint = static_cast<WheelJointPtr>(m_world.lock()->CreateJoint(&m_initialWheelJointStatus->getWheelJointDefinition()));
		}

	private:

		WorldWeakPtr m_world;

		InitialWheelJointStatusPtr m_initialWheelJointStatus;

		WheelJointPtr m_wheelJoint;

	};

	class Box2DInitialRevoluteJointStatus
	{
	public:

		using RevoluteJointDef = b2RevoluteJointDef;
		using Box2DBodyPtr = std::shared_ptr<Box2DBody>;

		void init(const Box2DBodyPtr& bodyA, const Box2DBodyPtr& bodyB, const Vec2& anchor)
		{
			m_revoluteJointDef.Initialize(bodyA->getBodyPtr(), bodyB->getBodyPtr(), ToB2Vec2(anchor));
		}

		void setBodyA(const Box2DBodyPtr& bodyA)
		{
			m_revoluteJointDef.bodyA = bodyA->getBodyPtr();
		}

		void setBodyB(const Box2DBodyPtr& bodyB)
		{
			m_revoluteJointDef.bodyB = bodyB->getBodyPtr();
		}

		void setLowerAngle(double lowerAngle)
		{
			m_revoluteJointDef.lowerAngle = static_cast<float32>(lowerAngle);
		}

		void setUpperAngle(double upperAngle)
		{
			m_revoluteJointDef.upperAngle = static_cast<float32>(upperAngle);
		}

		void setMotorSpeed(double motorSpeed)
		{
			m_revoluteJointDef.motorSpeed = static_cast<float32>(motorSpeed);
		}

		void setLocalAnchorA(const Vec2& anchor)
		{
			m_revoluteJointDef.localAnchorA = ToB2Vec2(anchor);
		}

		void setLocalAnchorB(const Vec2& anchor)
		{
			m_revoluteJointDef.localAnchorB = ToB2Vec2(anchor);
		}

		void setEnableMotoor(bool flag)
		{
			m_revoluteJointDef.enableMotor = flag;
		}

		void setEnableLimit(bool flag)
		{
			m_revoluteJointDef.enableLimit = flag;
		}

		void setMaxMotorTorque(double maxTorque)
		{
			m_revoluteJointDef.maxMotorTorque = static_cast<float32>(maxTorque);
		}

		void setCollideConnected(bool flag)
		{
			m_revoluteJointDef.collideConnected = flag;
		}

		const RevoluteJointDef& getRevoluteJointDefinition() const
		{
			return m_revoluteJointDef;
		}

	private:

		RevoluteJointDef m_revoluteJointDef;

	};

	class Box2DRevoluteJoint
	{
	public:
		using WorldWeakPtr = std::weak_ptr<b2World>;
		using InitialRevoluteJointStatusPtr = std::shared_ptr<Box2DInitialRevoluteJointStatus>;
		using RevoluteJointPtr = b2RevoluteJoint*;


		Box2DRevoluteJoint(const WorldWeakPtr& world)
			: m_world(world)
		{

		}

		virtual ~Box2DRevoluteJoint()
		{
			if (!m_world.expired())
				m_world.lock()->DestroyJoint(m_revoluteJoint);
		}

		void setInitialRevoluteJointStatus(const InitialRevoluteJointStatusPtr& initialRevoluteJointStatus)
		{
			m_initialRevoluteJointStatus = initialRevoluteJointStatus;
		}

		void setMotorSpeed(double motorSpeed)
		{
			m_revoluteJoint->SetMotorSpeed(static_cast<float32>(motorSpeed));
		}

		void init()
		{
			m_revoluteJoint = static_cast<RevoluteJointPtr>(m_world.lock()->CreateJoint(&m_initialRevoluteJointStatus->getRevoluteJointDefinition()));
		}

	private:

		WorldWeakPtr m_world;

		InitialRevoluteJointStatusPtr m_initialRevoluteJointStatus;

		RevoluteJointPtr m_revoluteJoint;

	};

	class Box2DInitialDistanceJointStatus
	{
	public:

		using DistanceJointDef = b2DistanceJointDef;
		using Box2DBodyPtr = std::shared_ptr<Box2DBody>;

		void init(const Box2DBodyPtr& bodyA, const Box2DBodyPtr& bodyB, const Vec2& anchorA, const Vec2& anchorB)
		{
			m_distanceJointDef.Initialize(bodyA->getBodyPtr(), bodyB->getBodyPtr(), ToB2Vec2(anchorA), ToB2Vec2(anchorB));
		}

		void setDampingRatio(double dampingRatio)
		{
			m_distanceJointDef.dampingRatio = static_cast<float32>(dampingRatio);
		}

		void setFrequencyHz(double frequencyHz)
		{
			m_distanceJointDef.frequencyHz = static_cast<float32>(frequencyHz);
		}

		const DistanceJointDef& getDistanceJointDefinition() const
		{
			return m_distanceJointDef;
		}

	private:

		DistanceJointDef m_distanceJointDef;
	};

	class Box2DDistanceJoint
	{
	public:
		using WorldWeakPtr = std::weak_ptr<b2World>;
		using InitialDistanceJointStatusPtr = std::shared_ptr<Box2DInitialDistanceJointStatus>;
		using DistanceJointPtr = b2DistanceJoint*;

		Box2DDistanceJoint(const WorldWeakPtr& world)
			: m_world(world)
		{

		}

		virtual ~Box2DDistanceJoint()
		{
			if (!m_world.expired())
				m_world.lock()->DestroyJoint(m_distanceJoint);
		}

		void setInitialDistanceJointStatus(const InitialDistanceJointStatusPtr& initialDistanceJointStatus)
		{
			m_initialDistanceJointStatus = initialDistanceJointStatus;
		}

		void init()
		{
			m_distanceJoint = static_cast<DistanceJointPtr>(m_world.lock()->CreateJoint(&m_initialDistanceJointStatus->getDistanceJointDefinition()));
		}

		void draw() const
		{
			Line(ToVec2(m_distanceJoint->GetAnchorA()), ToVec2(m_distanceJoint->GetAnchorB())).draw(0.1);
		}

	private:

		WorldWeakPtr m_world;

		InitialDistanceJointStatusPtr m_initialDistanceJointStatus;

		DistanceJointPtr m_distanceJoint;
	};

	class Box2DInitialPrismaticJointStatus
	{
	public:

		const b2PrismaticJointDef& getPrismaticJointDef() const
		{
			return m_prismaticJointDef;
		}

	private:
		

		b2PrismaticJointDef m_prismaticJointDef;
		

	};

	class Box2DPrismaticJoint
	{
	public:

		Box2DPrismaticJoint(const std::weak_ptr<b2World>& world)
			: m_world(world)
		{

		}

		virtual ~Box2DPrismaticJoint()
		{

		}

		void init()
		{
			m_prismaticJoint = static_cast<b2PrismaticJoint*>(m_world.lock()->CreateJoint(&m_initialDistanceJointDef->getPrismaticJointDef()));
		}

		void draw()
		{

		}

	private:

		std::shared_ptr<Box2DInitialPrismaticJointStatus> m_initialDistanceJointDef;

		std::weak_ptr<b2World> m_world;

		b2PrismaticJoint* m_prismaticJoint;
	};

	class Box2DManager
	{
	private:

		std::shared_ptr<b2World> m_world;

		int32 m_velocityIterations = 6;

		int32 m_positionIterations = 2;

	public:

		Box2DManager(const Vec2& gravity = Vec2(0.0, -9.8), int32 velocityIterations = 6, int32 positionIterations = 2)
			: m_world(std::make_shared<b2World>(ToB2Vec2(gravity)))
			, m_velocityIterations(velocityIterations)
			, m_positionIterations(positionIterations) {}

		virtual ~Box2DManager() {}

		void update(const double timeStep = 1.0 / 60.0)
		{
			m_world->Step(static_cast<float32>(timeStep), m_velocityIterations, m_positionIterations);
		}

		std::shared_ptr<Box2DBody> createBody() const
		{
			return std::make_shared<Box2DBody>(m_world);
		}

		std::shared_ptr<Box2DWheelJoint> createWheelJoint() const
		{
			return std::make_shared<Box2DWheelJoint>(m_world);
		}

		std::shared_ptr<Box2DDistanceJoint> createDistanceJoint() const
		{
			return std::make_shared<Box2DDistanceJoint>(m_world);
		}
	};

	void Box2DBody::setShape(const Line& line, const Box2DStatus& status)
	{
		auto shape = std::make_shared<Box2DLine>();
		shape->setLine(line);
		shape->setDensity(status.density);
		shape->setRestitution(status.restitution);
		shape->setFriction(status.friction);

		m_fixtureDefinitions.push_back(shape);
	}

	void Box2DBody::setShape(const LineString& line, const Box2DStatus& status)
	{
		auto shape = std::make_shared<Box2DLineString>();
		shape->setLineStringOpen(line);
		shape->setDensity(status.density);
		shape->setRestitution(status.restitution);
		shape->setFriction(status.friction);

		m_fixtureDefinitions.push_back(shape);
	}

	void Box2DBody::setShape(const RectF& rect, const Box2DStatus& status)
	{
		auto shape = std::make_shared<Box2DRect>();
		shape->setRect(rect);
		shape->setDensity(status.density);
		shape->setRestitution(status.restitution);
		shape->setFriction(status.friction);

		m_fixtureDefinitions.push_back(shape);
	}

	void Box2DBody::setShape(const Circle& circle, const Box2DStatus& status)
	{
		auto shape = std::make_shared<Box2DCircle>();
		shape->setRadius(circle.r);
		shape->setPosition(circle.center);
		shape->setDensity(status.density);
		shape->setRestitution(status.restitution);
		shape->setFriction(status.friction);

		m_fixtureDefinitions.push_back(shape);
	}

	void Box2DBody::setShape(const Polygon& polygon, const Box2DStatus& status)
	{
		Array<Triangle> triangles;

		triangles.reserve(polygon.num_triangles);

		for (const auto& triangle : polygon.triangles())
		{
			triangles.push_back(triangle);
		}

		for (const auto& triangle : triangles)
		{
			auto shape = std::make_shared<Box2DPolygon>();
			shape->setPolygon(Polygon{ triangle.p, 3 });
			shape->setDensity(status.density);
			shape->setRestitution(status.restitution);
			shape->setFriction(status.friction);

			m_fixtureDefinitions.push_back(shape);
		}
	}
}
