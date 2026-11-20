#include "KartUnrealTrackQuery.h"

#include "Components/PrimitiveComponent.h"
#include "Engine/HitResult.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "KartUnrealConversions.h"
#include "PhysicalMaterials/PhysicalMaterial.h"

void FKartUnrealTrackQuery::Setup(UWorld* InWorld, const AActor* InIgnoreActor)
{
	World = InWorld;
	IgnoreActor = InIgnoreActor;
}

bool FKartUnrealTrackQuery::SurfaceFromName(const FString& Text, const TCHAR* Prefix, kartsim::ESurfaceType& Out)
{
	using kartsim::ESurfaceType;
	if (!Text.StartsWith(Prefix))
	{
		return false;
	}
	const FString Name = Text.RightChop(FCString::Strlen(Prefix));
	for (int32 I = 0; I < static_cast<int32>(ESurfaceType::Count); ++I)
	{
		if (Name.Equals(UTF8_TO_TCHAR(kartsim::SurfaceName(static_cast<ESurfaceType>(I))), ESearchCase::IgnoreCase))
		{
			Out = static_cast<ESurfaceType>(I);
			return true;
		}
	}
	return false;
}

bool FKartUnrealTrackQuery::SurfaceFromTags(const TArray<FName>& Tags, kartsim::ESurfaceType& Out)
{
	for (const FName& Tag : Tags)
	{
		if (SurfaceFromName(Tag.ToString(), TEXT("KartSurface."), Out)) { return true; }
	}
	return false;
}

bool FKartUnrealTrackQuery::QueryGround(const kartsim::Vec3& Origin, double MaxUp, double MaxDown, kartsim::GroundHit& Out) const
{
	UWorld* W = World.Get();
	if (!W)
	{
		return false;
	}
	++TraceCount;
	const FVector Start = KartConv::ToUEPos(Origin + kartsim::Vec3(0.0, 0.0, MaxUp));
	const FVector End = KartConv::ToUEPos(Origin - kartsim::Vec3(0.0, 0.0, MaxDown));

	FCollisionQueryParams Params(SCENE_QUERY_STAT(KartGroundTrace), /*bTraceComplex*/ true);
	Params.bReturnPhysicalMaterial = true;
	if (const AActor* Ignore = IgnoreActor.Get())
	{
		Params.AddIgnoredActor(Ignore);
	}
	FHitResult Hit;
	if (!W->LineTraceSingleByChannel(Hit, Start, End, ECC_WorldStatic, Params))
	{
		return false;
	}
	Out.Point = KartConv::FromUEPos(Hit.ImpactPoint);
	Out.Normal = KartConv::FromUEDir(Hit.ImpactNormal).Normalized();
	// Surface resolution: 1) component tag "KartSurface.<Name>", 2) static mesh ASSET name containing
	// "KartSurface_<Name>" (convention for imported track art, e.g. SM_Track01_KartSurface_Grass),
	// 3) default asphalt. The naming convention is verified by -KartSurfaceSelfTest. (A physical-material
	// convention was also considered; early tests returned no material, but that turned out to be
	// asynchronous body creation, so that route is simply unverified rather than impossible.)
	kartsim::ESurfaceType Surface = kartsim::ESurfaceType::Asphalt;
	if (const UPrimitiveComponent* Comp = Hit.GetComponent())
	{
		if (!SurfaceFromTags(Comp->ComponentTags, Surface))
		{
			if (const UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(Comp))
			{
				if (const UStaticMesh* SM = SMC->GetStaticMesh())
				{
					const FString AssetName = SM->GetName();
					const int32 At = AssetName.Find(TEXT("KartSurface_"));
					if (At != INDEX_NONE)
					{
						FString Rest = AssetName.Mid(At);
						int32 Underscore = INDEX_NONE;
						// Accept a trailing "_Suffix" after the surface name (e.g. ..._KartSurface_Grass_02).
						if (Rest.Mid(12).FindChar(TEXT('_'), Underscore)) { Rest = Rest.Left(12 + Underscore); }
						SurfaceFromName(Rest, TEXT("KartSurface_"), Surface);
					}
				}
			}
		}
	}
	Out.Surface = Surface;
	Out.GripMultiplier = 1.0;
	return true;
}
